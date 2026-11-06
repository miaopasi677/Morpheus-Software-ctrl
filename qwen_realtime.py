"""Qwen-Audio 实时双工语音：麦克风 -> WebSocket -> 回答音频（边播边攒）。

模型：qwen-audio-3.1-realtime-flash
协议：WebSocket，事件驱动（不是 HTTP chat.completions）
    输入 PCM 16kHz / 16bit / 单声道，输出 PCM 24kHz / 16bit / 单声道
    客户端事件 session.update / input_audio_buffer.append
    服务端事件 response.audio.delta / response.done / response.audio_transcript.done
    鉴权 header: Authorization: Bearer <API_KEY>

一轮回复结束（response.done）时回调 on_reply(pcm24_bytes, transcript)，
调用方拿这段完整音频去重采样 16k -> A2F -> 口型 / PWM。

注意：A2F 用的是 wav2vec，需要整段音频才有上下文，所以当前架构下
口型会比声音晚一整句。要真·实时口型得换流式 A2F 或用文本转 viseme，
见 运行指南.md 的「延迟」一节。

    python qwen_realtime.py --check       只测 WS 建连，不录音不发音频
    python qwen_realtime.py               实时对话，回答存到 result/qwen/
    python qwen_realtime.py --selftest    不联网自检

依赖：pip install websockets pyaudio
API Key 从环境变量读，不要写进配置文件。
"""

import argparse
import asyncio
import base64
import json
import os
import pathlib
import sys
import time
import wave

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def load_config(path=None):
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def ws_url(cfg):
    q = cfg["qwen"]
    tpl = q["ws_url"]
    wid = q.get("workspace_id", "").strip()
    if "{WorkspaceId}" in tpl:
        if not wid or wid.startswith("填"):
            raise RuntimeError("hardware_config.json 里 qwen.workspace_id 还没填（百炼控制台的业务空间 ID）")
        tpl = tpl.replace("{WorkspaceId}", wid)
    sep = "&" if "?" in tpl else "?"
    return "%s%smodel=%s" % (tpl, sep, q["model"])


def pcm_to_wav(pcm, path, sr=24000):
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(p), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
        w.writeframes(pcm)
    return p


class RealtimeSession:
    def __init__(self, cfg, on_reply=None, api_key=None, play=True,
                 on_audio_delta=None, on_reply_done=None):
        self.cfg = cfg
        self.q = cfg["qwen"]
        self.on_reply = on_reply
        self.play = play
        # 全流式回调：每 100ms 的 16k PCM 调一次 on_audio_delta，
        # 一轮回复结束调 on_reply_done（用来 flush 尾部口型帧）。
        self.on_audio_delta = on_audio_delta
        self.on_reply_done = on_reply_done
        self.key = api_key or os.environ.get(self.q.get("api_key_env", "DASHSCOPE_API_KEY"))
        if not self.key:
            raise RuntimeError("请设置环境变量 %s" % self.q.get("api_key_env", "DASHSCOPE_API_KEY"))
        self.buf = bytearray()
        self.transcript = []
        self.cancelled = False
        self.pya = None
        self.mic = None
        self.spk = None
        self.pcm24_buf = bytearray()
        self.last_sample = 0.0
        self._feed_count = 0

    def _feed_audio(self, pcm24):
        """24k 音频流切成 100ms 块 -> 16k PCM，喂给 A2F。"""
        import numpy as np
        self.pcm24_buf.extend(pcm24)
        block = 4800  # 24000 * 2 * 0.1s
        while len(self.pcm24_buf) >= block:
            chunk = bytes(self.pcm24_buf[:block])
            del self.pcm24_buf[:block]
            arr = np.frombuffer(chunk, dtype="<i2").astype(np.float32)
            arr = np.concatenate([[self.last_sample], arr])
            self.last_sample = float(arr[-1])
            x = np.linspace(0.0, len(arr) - 1, 3200)  # 16k * 0.1s
            out = np.interp(x, np.arange(len(arr)), arr).astype("<i2").tobytes()
            if self.on_audio_delta:
                self.on_audio_delta(out)
            self._feed_count += 1

    def _open_audio(self):
        import pyaudio
        self.pya = pyaudio.PyAudio()
        self.mic = self.pya.open(format=pyaudio.paInt16, channels=1,
                                 rate=int(self.q.get("input_sample_rate", 16000)),
                                 input=True, frames_per_buffer=3200)
        if self.play:
            self.spk = self.pya.open(format=pyaudio.paInt16, channels=1,
                                     rate=int(self.q.get("output_sample_rate", 24000)),
                                     output=True)

    def _close_audio(self):
        for s in (self.mic, self.spk):
            if s:
                try:
                    s.stop_stream(); s.close()
                except Exception:
                    pass
        if self.pya:
            self.pya.terminate()

    async def _check(self):
        import websockets
        url = ws_url(self.cfg)
        headers = {"Authorization": "Bearer " + self.key,
                   "x-dashscope-dataInspection": "disable"}
        async with websockets.connect(url, additional_headers=headers,
                                      open_timeout=10) as ws:
            await ws.send(json.dumps({
                "type": "session.update",
                "session": {"modalities": ["text", "audio"],
                            "voice": self.q.get("voice", "longanqian_v3.1"),
                            "input_audio_format": "pcm",
                            "output_audio_format": "pcm",
                            "turn_detection": {"type": self.q.get("turn_detection", "smart_turn_v2")}},
            }))
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=8)
                ev = json.loads(msg)
                print("  WS 首个事件:", ev.get("type"), flush=True)
                if ev.get("type") == "error":
                    raise RuntimeError(ev["error"].get("message", str(ev)))
            except asyncio.TimeoutError:
                print("  WS 已连上（8 秒内无事件，通常也正常）", flush=True)
        return True

    async def _run(self):
        import websockets
        url = ws_url(self.cfg)
        headers = {"Authorization": "Bearer " + self.key,
                   "x-dashscope-dataInspection": "disable"}
        self._open_audio()
        try:
            async with websockets.connect(url, additional_headers=headers,
                                          open_timeout=10) as ws:
                await ws.send(json.dumps({
                    "type": "session.update",
                    "session": {
                        "modalities": ["text", "audio"],
                        "voice": self.q.get("voice", "longanqian_v3.1"),
                        "instructions": self.q.get("system_prompt", ""),
                        "input_audio_format": "pcm",
                        "output_audio_format": "pcm",
                        "turn_detection": {"type": self.q.get("turn_detection", "smart_turn_v2")},
                    },
                }))
                print("  已连接，开始说话（Ctrl+C 结束）", flush=True)

                async def mic_loop():
                    while True:
                        data = await asyncio.to_thread(self.mic.read, 3200, False)
                        await ws.send(json.dumps({
                            "type": "input_audio_buffer.append",
                            "audio": base64.b64encode(data).decode(),
                        }))
                        await asyncio.sleep(0.01)

                async def recv_loop():
                    async for msg in ws:
                        ev = json.loads(msg)
                        t = ev.get("type")
                        if t == "response.created":
                            self.buf.clear(); self.transcript.clear()
                            self.cancelled = False
                        elif t == "input_audio_buffer.speech_started":
                            # 用户打断：停止播放，丢弃本轮残余音频
                            self.cancelled = True
                            self.buf.clear()
                            await ws.send(json.dumps({"type": "response.cancel"}))
                        elif t == "response.audio.delta" and not self.cancelled:
                            pcm = base64.b64decode(ev.get("delta", ""))
                            self.buf.extend(pcm)
                            if self.spk:
                                await asyncio.to_thread(self.spk.write, pcm)
                            if self.on_audio_delta:
                                await asyncio.to_thread(self._feed_audio, pcm)
                        elif t == "response.audio_transcript.done":
                            self.transcript.append(ev.get("transcript", ""))
                        elif t == "conversation.item.input_audio_transcription.completed":
                            print("  [你]", ev.get("transcript", ""), flush=True)
                        elif t == "response.done":
                            text = "".join(self.transcript).strip()
                            if self.buf and not self.cancelled:
                                print("  [AI]", text[:120], flush=True)
                                if self.on_reply:
                                    await asyncio.to_thread(self.on_reply, bytes(self.buf), text)
                                else:
                                    out = ROOT / self.q.get("save_dir", "result/qwen")
                                    pcm_to_wav(bytes(self.buf), out / "reply.wav",
                                               int(self.q.get("output_sample_rate", 24000)))
                            if self.on_reply_done and not self.cancelled:
                                await asyncio.to_thread(self.on_reply_done)
                            self.buf.clear(); self.transcript.clear()
                            self.pcm24_buf.clear()
                        elif t == "error":
                            print("  [错误]", ev.get("error", {}).get("message", ev), flush=True)

                await asyncio.gather(mic_loop(), recv_loop())
        finally:
            self._close_audio()

    def check(self):
        return asyncio.run(self._check())

    def run(self):
        return asyncio.run(self._run())


def selftest():
    cfg = load_config()
    q = cfg["qwen"]
    assert q["model"] == "qwen-audio-3.1-realtime-flash", q["model"]
    assert "{WorkspaceId}" in q["ws_url"] or q["ws_url"].startswith("wss://")
    assert q.get("input_sample_rate") == 16000
    assert q.get("output_sample_rate") == 24000
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = pcm_to_wav(b"\x01\x02" * 2400, pathlib.Path(td) / "r.wav", 24000)
        with wave.open(str(p), "rb") as w:
            assert w.getframerate() == 24000 and w.getnchannels() == 1
    print("QWEN_REALTIME_SELFTEST_OK 模型/采样率/PCM->wav 均正确（未联网）")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--check", action="store_true", help="只测 WebSocket 建连")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    cfg = load_config(args.config)
    if args.check:
        s = RealtimeSession(cfg, play=False)
        s.check()
        print("QWEN_WS_OK 连接与 session.update 正常")
        return
    RealtimeSession(cfg).run()


if __name__ == "__main__":
    main()
