"""全流式入口：说话 -> Qwen -> 流式 A2F -> Blender 形态键 -> 33 路 PWM -> 舵机。

架构（PWM 只有一个来源：Blender 里的 morpheus_arkit_bridge.py）：

    Qwen WebSocket 音频 delta (24k)
      ├─ 立即播放（声音低延迟）
      └─ 切 100ms 块重采样 16k -> StreamingA2F 滑窗推理 -> 52 维 ARKit 帧
           -> UDP 9876 -> Blender 形态键
                -> morpheus_arkit_bridge.apply_frame()
                     -> 33 根骨 + Scene["morpheus_pwm"]
                     -> UDP 9878 -> 本进程 PwmListener -> 驱动板

    python run.py --selftest                 全链路自检，不联网、不接硬件
    python run.py --check                   只测 Qwen WebSocket 建连
    python run.py --dry-run                 真对话 + 真推理，PWM 只打印不下发
    python run.py                           正式跑
    python run.py --mode file --wav audio/happy.wav   不联网，流式演练后半段

Ctrl+C 自动把 33 路舵机送回 Table V 中立值。
"""

import argparse
import json
import os
import pathlib
import socket
import sys
import threading
import time
import wave

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def log(msg):
    print("[run] " + msg, flush=True)


def load_config(path=None):
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def preflight(cfg, need_qwen=True):
    problems = []
    q = cfg["qwen"]
    if need_qwen and not os.environ.get(q.get("api_key_env", "DASHSCOPE_API_KEY")):
        problems.append("缺 API Key：设置环境变量 %s" % q.get("api_key_env", "DASHSCOPE_API_KEY"))
    if need_qwen and "{WorkspaceId}" in q.get("ws_url", "") and (
            not q.get("workspace_id") or str(q.get("workspace_id")).startswith("填")):
        problems.append("hardware_config.json 里 qwen.workspace_id 没填（百炼控制台的业务空间 ID）")
    if not (ROOT / cfg["a2f"]["model_path"]).exists():
        problems.append("缺 A2F 权重：" + cfg["a2f"]["model_path"])
    if not (ROOT / cfg["blend"]).exists():
        problems.append("缺 blend 文件：" + cfg["blend"])
    if not pathlib.Path(cfg["blender_exe"]).exists():
        problems.append("Blender 路径不对：" + cfg["blender_exe"])
    return problems


class PwmListener(threading.Thread):
    """监听 Blender 发来的 33 路 PWM，收到就下发给驱动板。"""

    def __init__(self, out, cfg):
        super().__init__(daemon=True)
        st = cfg.get("streaming", {})
        self.host = st.get("pwm_udp_host", "127.0.0.1")
        self.port = int(st.get("pwm_udp_port", 9878))
        self.out = out
        self.stop = threading.Event()
        self.count = 0
        self.sock = None

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host if self.host not in ("0.0.0.0", "") else "", self.port))
        self.sock.settimeout(0.2)
        log("PWM 监听 UDP %s:%d（来自 Blender）" % (self.host, self.port))
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                pwm = json.loads(data)["pwm"]
                if len(pwm) == 33:
                    self.out.send(pwm)
                    self.count += 1
            except Exception as e:
                log("PWM 包解析失败: %s" % e)
        if self.sock:
            self.sock.close()

    def close(self):
        self.stop.set()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


def play_wav(path):
    try:
        import numpy as np
        import sounddevice as sd
    except ImportError:
        log("没装 sounddevice，跳过播放")
        return
    with wave.open(str(path), "rb") as w:
        sr, n, width = w.getframerate(), w.getnchannels(), w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if width == 2:
        sd.play(np.frombuffer(raw, dtype="<i2").reshape(-1, n), sr)


def build_streaming(cfg, dry_run=False):
    """构造流式 A2F + UDP 推流器，返回 (a2f, streamer, on_audio_delta, on_done)。"""
    import a2f_infer
    import stream_arkit

    a2f = a2f_infer.StreamingA2F(cfg)
    streamer = stream_arkit.ArkitStreamer(cfg, dry_run=dry_run)
    lock = threading.Lock()
    stats = {"frames": 0, "chunks": 0}

    def on_audio_delta(pcm16):
        with lock:
            frames = a2f.push(pcm16)
            stats["chunks"] += 1
            if len(frames):
                streamer.push(frames)
                stats["frames"] += len(frames)
            if stats["chunks"] % 10 == 0:
                log("流式口型 %d 帧 / %d 块" % (stats["frames"], stats["chunks"]))

    def on_done():
        with lock:
            tail = a2f.flush()
            if len(tail):
                streamer.push(tail)
                stats["frames"] += len(tail)
            log("本轮口型共 %d 帧" % stats["frames"])
    return a2f, streamer, on_audio_delta, on_done, stats


def run_file_mode(wav_path, cfg, out, dry_run, play=True):
    """不联网：把现成 wav 切成 100ms 块，按流式路径演练一遍。"""
    import a2f_infer
    with wave.open(str(wav_path), "rb") as w:
        sr, n, width, raw = (w.getframerate(), w.getnchannels(),
                             w.getsampwidth(), w.readframes(w.getnframes()))
    if width != 2:
        raise ValueError("只支持 16bit wav")
    import numpy as np
    s = np.frombuffer(raw, dtype="<i2").astype(np.float32)
    if n > 1:
        s = s.reshape(-1, n).mean(axis=1)
    if sr != 16000:
        x = np.linspace(0, len(s) - 1, int(len(s) * 16000 / sr))
        s = np.interp(x, np.arange(len(s)), s)
    pcm16 = s.astype("<i2").tobytes()
    _, streamer, on_delta, on_done, stats = build_streaming(cfg, dry_run)
    if play:
        play_wav(wav_path)
    block = 3200  # 16000 * 2 * 0.1s
    for i in range(0, len(pcm16), block):
        on_delta(pcm16[i:i + block])
    on_done()
    streamer.finish()
    return stats["frames"]


def selftest():
    import a2f_infer, qwen_realtime, stream_arkit
    import servo_output, calibrate_channels, servo_pwm
    log("各模块自检…")
    a2f_infer.selftest()
    qwen_realtime.selftest()
    stream_arkit.selftest()
    servo_output.selftest()
    calibrate_channels.selftest()
    servo_pwm.selftest()

    # 流式推流器 dry-run：用假 ARKit 帧验证重排与节拍
    import numpy as np
    cfg = load_config()
    s = stream_arkit.ArkitStreamer(cfg, dry_run=True)
    frames = np.zeros((4, 52), dtype=np.float32)
    frames[:, a2f_infer.MODEL_BS_LIST.index("jawOpen")] = 0.8
    assert s.push(frames) == 4
    assert s.finish() == 4

    # PWM 监听链路：模拟 Blender 发一个包
    cfg["servo"]["backend"] = "dry_run"
    out = servo_output.ServoOutput(cfg, servo_output.DryRunBackend())
    out.open()
    lis = PwmListener(out, cfg)
    lis.start()
    time.sleep(0.15)
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.sendto(json.dumps({"pwm": servo_output.neutral_pwm()}).encode(),
                 (cfg["streaming"]["pwm_udp_host"],
                  int(cfg["streaming"]["pwm_udp_port"])))
    probe.close()
    time.sleep(0.35)
    lis.close()
    lis.join(timeout=1)
    out.close()
    assert lis.count >= 1, "没收到 Blender 发来的 PWM 包"
    print("RUN_SELFTEST_OK 全流式链路自检通过（未联网、未接硬件、未加载模型）")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["realtime", "file"], default="realtime")
    ap.add_argument("--wav", help="file 模式用的现成 wav")
    ap.add_argument("--config")
    ap.add_argument("--check", action="store_true", help="只测 Qwen WebSocket 建连")
    ap.add_argument("--dry-run", action="store_true", help="PWM 只打印不下发")
    ap.add_argument("--no-play", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    cfg = load_config(args.config)
    if args.dry_run:
        cfg["servo"]["backend"] = "dry_run"

    if args.check:
        from qwen_realtime import RealtimeSession
        RealtimeSession(cfg, play=False).check()
        print("QWEN_WS_OK 连接与 session.update 正常")
        return 0

    problems = preflight(cfg, need_qwen=(args.mode == "realtime"))
    if problems:
        for p in problems:
            log("✗ " + p)
        log("先按 运行指南.md 把这几项配好")
        return 1

    from servo_output import ServoOutput, DryRunBackend
    out = ServoOutput(cfg, DryRunBackend() if args.dry_run else None).open()
    st_cfg = cfg.get("streaming", {})
    listener = None
    if st_cfg.get("pwm_from_blender", True):
        listener = PwmListener(out, cfg)
        listener.start()
        log("PWM 由 Blender 计算并通过 UDP 送来；本进程只负责下发")
        log("⚠ 请确认 Blender 已打开并 Run Script 了 morpheus_arkit_bridge.py")
    try:
        if args.mode == "realtime":
            from qwen_realtime import RealtimeSession
            _, streamer, on_delta, on_done, _ = build_streaming(cfg, args.dry_run)
            play = bool(st_cfg.get("play_audio_realtime", True)) and not args.no_play
            RealtimeSession(cfg, on_audio_delta=on_delta,
                            on_reply_done=on_done, play=play).run()
            streamer.finish()
        else:
            if not args.wav:
                log("file 模式需要 --wav 路径")
                return 1
            run_file_mode(args.wav, cfg, out, args.dry_run, not args.no_play)
    except KeyboardInterrupt:
        log("已中断")
    finally:
        if listener:
            listener.close()
        out.close()
        log("33 路已回中立")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
