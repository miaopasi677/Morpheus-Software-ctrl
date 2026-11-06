"""语音对话：麦克风 / 本地 wav / 文本 -> Qwen Omni -> 回答音频 wav。

Qwen 只负责"听懂 + 说出来"，它不输出 ARKit 系数。回答音频再交给
a2f_infer.py 生成 52 维口型，所以这条链路是：

    说话 -> Qwen( audio -> audio ) -> reply.wav(24k)
         -> resample 16k -> a2f_infer.py -> (T,52)
         -> stream_arkit.py -> Blender -> 33 路 PWM -> 舵机

接口（阿里云 DashScope OpenAI 兼容）：
    输入音频  {"type":"input_audio","input_audio":{"data":"data:audio/wav;base64,...","format":"wav"}}
    输出      modalities=["text","audio"], audio={"voice":"Tina","format":"wav"}, stream=True
    流式      chunk.choices[0].delta.audio["data"] (base64), ["transcript"]

API Key 从环境变量读，不写进配置文件：
    Windows PowerShell:  $env:DASHSCOPE_API_KEY="sk-xxx"
    Git Bash:            export DASHSCOPE_API_KEY=sk-xxx

    python qwen_talk.py --text "你好"           # 用文本试通 API
    python qwen_talk.py --wav audio/happy.wav   # 用现成录音，不用麦克风
    python qwen_talk.py                          # 麦克风录音（需 sounddevice）
    python qwen_talk.py --selftest               # 不联网自检
"""

import argparse
import base64
import json
import os
import pathlib
import struct
import sys
import wave

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def load_config(path=None):
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def record_wav(path, seconds=5, sr=16000):
    import numpy as np
    import sounddevice as sd
    print("  录音 %d 秒…" % seconds, flush=True)
    audio = sd.rec(int(seconds * sr), samplerate=sr, channels=1, dtype="int16")
    sd.wait()
    pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
        w.writeframes(audio.tobytes())
    return path


def pcm_to_wav(pcm_bytes, path, sr=24000):
    """Qwen 流式返回的可能是裸 PCM16，也可能是完整 wav，两种都兼容。"""
    p = pathlib.Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if pcm_bytes[:4] == b"RIFF":
        p.write_bytes(pcm_bytes)
    else:
        with wave.open(str(p), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(sr)
            w.writeframes(pcm_bytes)
    return p


def chat(cfg, wav=None, text=None, out_dir=None):
    from openai import OpenAI

    q = cfg["qwen"]
    key = os.environ.get(q.get("api_key_env", "DASHSCOPE_API_KEY"))
    if not key:
        raise RuntimeError("没找到 API Key，请先设置环境变量 %s" %
                           q.get("api_key_env", "DASHSCOPE_API_KEY"))
    client = OpenAI(api_key=key, base_url=q["base_url"])

    if wav:
        b64 = base64.b64encode(pathlib.Path(wav).read_bytes()).decode()
        content = [
            {"type": "input_audio", "input_audio": {
                "data": "data:audio/wav;base64," + b64, "format": "wav"}},
        ]
    else:
        content = text or "你好，介绍一下你自己。"

    messages = [{"role": "system", "content": q.get("system_prompt", "")},
                {"role": "user", "content": content}]

    completion = client.chat.completions.create(
        model=q["model"],
        messages=messages,
        modalities=["text", "audio"],
        audio={"voice": q.get("voice", "Tina"), "format": "wav"},
        stream=True,
        stream_options={"include_usage": True},
    )

    buf = bytearray()
    transcript = []
    for chunk in completion:
        if not chunk.choices:
            continue
        delta = chunk.choices[0].delta
        audio = getattr(delta, "audio", None)
        if audio:
            d = audio.get("data") if isinstance(audio, dict) else getattr(audio, "data", None)
            if d:
                buf.extend(base64.b64decode(d))
            t = audio.get("transcript") if isinstance(audio, dict) else getattr(audio, "transcript", None)
            if t:
                transcript.append(t)
        c = getattr(delta, "content", None)
        if c:
            transcript.append(c)
    if not buf:
        raise RuntimeError("Qwen 没返回音频，检查模型名 / modalities / 账号权限")

    out = pathlib.Path(out_dir or (ROOT / q.get("save_dir", "result/qwen")))
    out.mkdir(parents=True, exist_ok=True)
    wav24 = pcm_to_wav(bytes(buf), out / "reply.wav", int(q.get("output_sample_rate", 24000)))
    (out / "reply.txt").write_text("".join(transcript).strip(), encoding="utf-8")

    # A2F 需要 16k 单声道
    from a2f_infer import resample_wav
    wav16 = resample_wav(wav24, out / "reply_16k.wav", 16000)
    print("  Qwen:", "".join(transcript).strip()[:120], flush=True)
    print("  音频:", wav24, "/", wav16, flush=True)
    return {"wav": str(wav24), "wav16": str(wav16), "text": "".join(transcript).strip()}


def selftest():
    """备用 HTTP 通道的自检；当前配置是 realtime 时跳过联网参数检查。"""
    cfg = load_config()
    q = cfg["qwen"]
    if "base_url" not in q:
        print("QWEN_TALK_SELFTEST_SKIP 当前配置是 realtime；本文件是 HTTP Omni 备用通道")
        return True
    assert q["model"] and q["base_url"] and q["voice"]
    # 造一段假 PCM，验证 wav 封装 + 24k->16k 重采样
    import tempfile
    from a2f_infer import resample_wav
    with tempfile.TemporaryDirectory() as td:
        p = pcm_to_wav(b"\x00\x00" * 2400, pathlib.Path(td) / "r.wav", 24000)
        with wave.open(str(p), "rb") as w:
            assert w.getframerate() == 24000 and w.getnchannels() == 1
        b = resample_wav(p, pathlib.Path(td) / "r16.wav", 16000)
        with wave.open(str(b), "rb") as w:
            assert w.getframerate() == 16000
        # 已经是 wav 的情况应原样保留
        p2 = pcm_to_wav(p.read_bytes(), pathlib.Path(td) / "r2.wav", 24000)
        assert p2.read_bytes()[:4] == b"RIFF"
    assert (ROOT / cfg["a2f"]["model_path"]).exists()
    print("QWEN_SELFTEST_OK config/wav/16k OK（未联网；联网测试用 --text）")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text")
    ap.add_argument("--wav")
    ap.add_argument("--mic", action="store_true")
    ap.add_argument("--seconds", type=float)
    ap.add_argument("--config")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    cfg = load_config(args.config)
    q = cfg["qwen"]
    out = ROOT / q.get("save_dir", "result/qwen")
    wav = args.wav
    if args.mic or (not args.text and not wav):
        wav = record_wav(out / "mic.wav",
                         float(args.seconds or q.get("mic_seconds", 5)),
                         int(q.get("mic_sample_rate", 16000)))
    result = chat(cfg, wav=wav, text=args.text)
    print("QWEN_OK", json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
