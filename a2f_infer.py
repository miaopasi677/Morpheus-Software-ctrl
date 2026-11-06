"""把一段 wav 转成 52 维 ARKit 系数 (T,52) @30fps。

就是 demo.py 里的推理部分，去掉了渲染 mp4 和随机眨眼，做成可 import 的函数。
原 demo.py 保持论文原样不动。

单独用：
    python a2f_infer.py result/qwen/reply.wav --out result/reply.npy
自检（不需要模型权重）：
    python a2f_infer.py --selftest
"""

import argparse
import os
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

MODEL_BS_LIST = [
    "browDownLeft", "browDownRight", "browInnerUp", "browOuterUpLeft", "browOuterUpRight",
    "cheekPuff", "cheekSquintLeft", "cheekSquintRight", "eyeBlinkLeft", "eyeBlinkRight",
    "eyeLookDownLeft", "eyeLookDownRight", "eyeLookInLeft", "eyeLookInRight",
    "eyeLookOutLeft", "eyeLookOutRight", "eyeLookUpLeft", "eyeLookUpRight",
    "eyeSquintLeft", "eyeSquintRight", "eyeWideLeft", "eyeWideRight",
    "jawForward", "jawLeft", "jawOpen", "jawRight", "mouthClose", "mouthDimpleLeft",
    "mouthDimpleRight", "mouthFrownLeft", "mouthFrownRight", "mouthFunnel", "mouthLeft",
    "mouthLowerDownLeft", "mouthLowerDownRight", "mouthPressLeft", "mouthPressRight",
    "mouthPucker", "mouthRight", "mouthRollLower", "mouthRollUpper", "mouthShrugLower",
    "mouthShrugUpper", "mouthSmileLeft", "mouthSmileRight", "mouthStretchLeft",
    "mouthStretchRight", "mouthUpperUpLeft", "mouthUpperUpRight", "noseSneerLeft",
    "noseSneerRight", "tongueOut",
]


def load_config(path=None):
    import json
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def _args(cfg):
    from types import SimpleNamespace
    a = cfg["a2f"]
    return SimpleNamespace(
        bs_dim=52, feature_dim=832, period=30,
        emo_guide=True, max_seq_len=5000, num_workers=0, batch_size=1,
        device=a["device"], model_path=str(ROOT / a["model_path"]),
    )


_MODEL_CACHE = {}


def get_model(cfg=None, device=None):
    """模型只加载一次，实时对话每轮都重载会卡好几秒。"""
    import torch
    from a2fmodel import A2BModel

    cfg = cfg or load_config()
    a = cfg["a2f"]
    dev = device or a["device"]
    key = (dev, str(ROOT / a["model_path"]))
    if key in _MODEL_CACHE:
        return _MODEL_CACHE[key][0], dev
    args = _args(cfg)
    args.device = dev
    model = A2BModel(args)
    state = torch.load(args.model_path, map_location="cpu")
    model.load_state_dict(state, strict=False)
    model = model.to(dev).eval()
    _MODEL_CACHE[key] = (model, dev)
    return model, dev


def infer_array(speech, cfg=None, device=None):
    """16k float32 波形 -> (T,52)，列序 = MODEL_BS_LIST。流式推理用这个，不落盘。"""
    import torch
    from scipy.signal import savgol_filter

    cfg = cfg or load_config()
    a = cfg["a2f"]
    model, dev = get_model(cfg, device)
    audio = torch.FloatTensor(np.asarray(speech, dtype=np.float32)).unsqueeze(0).to(dev)
    level = torch.tensor([1]).to(dev)
    person = torch.tensor([0]).to(dev)
    with torch.no_grad():
        pred = model.predict(audio, level, person)
    out = pred.squeeze().detach().cpu().numpy()
    if out.ndim == 1:
        out = out[None, :]

    if a.get("post_processing", True):
        smoothed = np.zeros_like(out)
        for i in range(out.shape[1]):
            smoothed[:, i] = (savgol_filter(out[:, i], 5, 2)
                              if out.shape[0] >= 5 else out[:, i])
        # demo.py 清零 8/9 列（眨眼）后随机插入；流式链路里不插随机眨眼
        smoothed[:, 8] = 0.0
        smoothed[:, 9] = 0.0
        out = smoothed
    return np.clip(out, 0.0, 1.0).astype(np.float32)


def infer_wav(wav_path, cfg=None, device=None):
    """wav -> (T,52)，列序 = MODEL_BS_LIST（demo.py 的输出顺序）。"""
    import librosa
    speech, _ = librosa.load(str(wav_path), sr=16000)
    return infer_array(speech, cfg, device)


class StreamingA2F:
    """滑窗流式 A2F：边收 16k PCM 边产出 30fps 的 ARKit 帧。

    wav2vec2 需要上下文，所以每个 chunk 前面拼一段上一块的尾部再推理，
    只取属于新 chunk 的那几帧输出。这样块与块之间口型是连续的，
    延迟 = 一个 chunk + 一次推理，而不是等整句说完。

        st = StreamingA2F(cfg)
        frames = st.push(pcm16_bytes)   # -> (N,52) 或空
        tail   = st.flush()             # 一句话结束时冲出剩余帧
    """

    def __init__(self, cfg=None, device=None):
        cfg = cfg or load_config()
        st = cfg.get("streaming", {})
        self.sr = 16000
        self.fps = int(st.get("a2f_fps", 30))
        self.chunk_n = int(float(st.get("chunk_s", 0.5)) * self.sr)
        self.ctx_n = int(float(st.get("context_s", 0.3)) * self.sr)
        self.cfg = cfg
        self.device = device
        self.buf = np.zeros(0, dtype=np.float32)
        self.tail = np.zeros(0, dtype=np.float32)

    def _infer(self, window, new_samples):
        if len(window) < 400:
            return np.zeros((0, 52), dtype=np.float32)
        bs = infer_array(window, self.cfg, self.device)
        n_new = max(1, int(round(new_samples / self.sr * self.fps)))
        return bs[-n_new:] if len(bs) else np.zeros((0, 52), dtype=np.float32)

    def push(self, pcm16_bytes):
        s = np.frombuffer(pcm16_bytes, dtype="<i2").astype(np.float32) / 32768.0
        self.buf = np.concatenate([self.buf, s])
        out = []
        while len(self.buf) >= self.chunk_n:
            chunk = self.buf[:self.chunk_n]
            self.buf = self.buf[self.chunk_n:]
            window = np.concatenate([self.tail, chunk])
            out.append(self._infer(window, self.chunk_n))
            self.tail = window[-self.ctx_n:] if self.ctx_n else np.zeros(0, dtype=np.float32)
        return np.concatenate(out, axis=0) if out else np.zeros((0, 52), dtype=np.float32)

    def flush(self):
        if len(self.buf) < 400:
            self.buf = np.zeros(0, dtype=np.float32)
            return np.zeros((0, 52), dtype=np.float32)
        window = np.concatenate([self.tail, self.buf])
        n = len(self.buf)
        self.buf = np.zeros(0, dtype=np.float32)
        return self._infer(window, n)


def resample_wav(src, dst, target_sr=16000):
    """Qwen 输出 24k wav -> A2F 需要的 16k wav。

    不用 audioop：Blender 5.x 自带 Python 3.13 已经移除了它。
    这里用 numpy 线性插值，对 16k 语音输入足够。
    """
    import wave
    import numpy as np
    with wave.open(str(src), "rb") as w:
        n, sr, width = w.getnchannels(), w.getframerate(), w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if sr == target_sr and n == 1 and width == 2:
        pathlib.Path(dst).parent.mkdir(parents=True, exist_ok=True)
        pathlib.Path(dst).write_bytes(pathlib.Path(src).read_bytes())
        return dst
    if width != 2:
        raise ValueError("只支持 16bit PCM，收到 %d 字节/采样" % width)
    samples = np.frombuffer(raw, dtype="<i2").astype(np.float32)
    if n > 1:
        samples = samples.reshape(-1, n).mean(axis=1)
    if sr != target_sr and len(samples) > 1:
        n_out = max(1, int(round(len(samples) * target_sr / sr)))
        x = np.linspace(0.0, len(samples) - 1, n_out)
        samples = np.interp(x, np.arange(len(samples)), samples)
    out = np.clip(samples, -32768, 32767).astype("<i2").tobytes()
    pathlib.Path(dst).parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(dst), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(target_sr)
        w.writeframes(out)
    return dst


def selftest():
    assert len(MODEL_BS_LIST) == 52
    import json
    cfg = load_config()
    assert set(cfg) >= {"qwen", "a2f", "stream", "servo"}
    assert (ROOT / cfg["a2f"]["model_path"]).exists(), "模型权重不存在"
    # 生成一小段 24k wav，验证重采样
    import wave, tempfile
    with tempfile.TemporaryDirectory() as td:
        a = pathlib.Path(td) / "a.wav"
        b = pathlib.Path(td) / "b.wav"
        with wave.open(str(a), "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000)
            w.writeframes(b"\x00\x00" * 2400)
        resample_wav(a, b, 16000)
        with wave.open(str(b), "rb") as w:
            assert w.getframerate() == 16000, w.getframerate()
            assert w.getnchannels() == 1
    print("A2F_SELFTEST_OK 52 keys, weight present, 24k->16k ok")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav", nargs="?")
    ap.add_argument("--out")
    ap.add_argument("--config")
    ap.add_argument("--device")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.wav:
        ap.error("需要 wav 路径，或用 --selftest")
    cfg = load_config(args.config)
    data = infer_wav(args.wav, cfg, args.device)
    out = args.out or str(ROOT / cfg["a2f"]["result_path"] /
                          (pathlib.Path(args.wav).stem + ".npy"))
    pathlib.Path(out).parent.mkdir(parents=True, exist_ok=True)
    np.save(out, data)
    print("A2F_OK", out, data.shape)


if __name__ == "__main__":
    main()
