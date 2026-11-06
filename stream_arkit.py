"""把 (T,52) ARKit 序列按 30fps 推给 Blender 里的 blender_receiver.py。

坑：demo.py 输出列序是 MODEL_BS_LIST，但 blend 里 receiver 用的是 ARKIT_NAMES，
两者顺序不同。这里做一次显式重排，不能按下标直接发。

协议（与 embedded_blender_receiver.py 一致）：
    MAGIC   = b"A2F\\x00"
    META    = MAGIC + 0x03 + json({"fps":30})
    FRAME   = MAGIC + 0x01 + struct.pack("!4sB I d I", MAGIC,1,idx,ts,52) + 52 x float32
    STOP    = MAGIC + 0x02
注意 HEADER 里已经包含 MAGIC，所以 FRAME 不需要再额外拼一次 MAGIC。

单独用：
    python stream_arkit.py result/reply.npy
自检（不发包）：
    python stream_arkit.py --selftest
"""

import argparse
import json
import pathlib
import socket
import struct
import sys
import time

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

MAGIC = b"A2F\x00"
TYPE_FRAME, TYPE_STOP, TYPE_META = 0x01, 0x02, 0x03
HEADER_FMT = "!4sB I d I"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# blend 内 blender_receiver.py 的 ARKIT_NAMES，顺序必须逐字一致
ARKIT_NAMES = [
    "eyeBlinkLeft", "eyeLookDownLeft", "eyeLookInLeft",
    "eyeLookOutLeft", "eyeLookUpLeft", "eyeSquintLeft",
    "eyeWideLeft", "eyeBlinkRight", "eyeLookDownRight",
    "eyeLookInRight", "eyeLookOutRight", "eyeLookUpRight",
    "eyeSquintRight", "eyeWideRight", "jawForward",
    "jawLeft", "jawRight", "jawOpen",
    "mouthClose", "mouthFunnel", "mouthPucker",
    "mouthLeft", "mouthRight", "mouthSmileLeft",
    "mouthSmileRight", "mouthFrownLeft", "mouthFrownRight",
    "mouthDimpleLeft", "mouthDimpleRight", "mouthStretchLeft",
    "mouthStretchRight", "mouthRollLower", "mouthRollUpper",
    "mouthShrugLower", "mouthShrugUpper", "mouthPressLeft",
    "mouthPressRight", "mouthLowerDownLeft", "mouthLowerDownRight",
    "mouthUpperUpLeft", "mouthUpperUpRight", "browDownLeft",
    "browDownRight", "browInnerUp", "browOuterUpLeft",
    "browOuterUpRight", "cheekPuff", "cheekSquintLeft",
    "cheekSquintRight", "noseSneerLeft", "noseSneerRight",
    "tongueOut",
]


def load_config(path=None):
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def reorder(bs):
    """(T,52) MODEL_BS_LIST 顺序 -> (T,52) ARKIT_NAMES 顺序。"""
    from a2f_infer import MODEL_BS_LIST
    assert len(MODEL_BS_LIST) == 52 and len(ARKIT_NAMES) == 52
    assert set(MODEL_BS_LIST) == set(ARKIT_NAMES), "两套 ARKit 名单不一致"
    idx = [MODEL_BS_LIST.index(n) for n in ARKIT_NAMES]
    return np.asarray(bs, dtype=np.float32)[:, idx]


def frame_packet(weights, idx, ts):
    w = [float(max(0.0, min(1.0, v))) for v in weights]
    return struct.pack(HEADER_FMT, MAGIC, TYPE_FRAME, idx, ts, 52) + struct.pack("!52f", *w)


def send_stream(bs, cfg=None, dry_run=False, progress=True, start_at=None):
    cfg = cfg or load_config()
    s = cfg["stream"]
    frames = reorder(bs)
    fps = float(s.get("fps", 30))
    dt = 1.0 / fps
    sock = None
    if not dry_run:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(MAGIC + bytes([TYPE_META]) +
                    json.dumps({"fps": fps}).encode(), (s["host"], s["port"]))
    t0 = start_at if start_at is not None else time.perf_counter()
    for i, f in enumerate(frames):
        if sock:
            sock.sendto(frame_packet(f, i, time.time()), (s["host"], s["port"]))
        elif not dry_run:
            pass
        if progress and i % 30 == 0:
            print("  stream frame %d/%d" % (i, len(frames)), flush=True)
        target = t0 + (i + 1) * dt
        wait = target - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
    if sock:
        sock.sendto(MAGIC + bytes([TYPE_STOP]), (s["host"], s["port"]))
        sock.close()
    return len(frames)


class ArkitStreamer:
    """流式推送：A2F 每产出几帧就立刻按 30fps 节拍发走，不等整句。"""

    def __init__(self, cfg=None, dry_run=False):
        cfg = cfg or load_config()
        s = cfg["stream"]
        self.fps = float(s.get("fps", 30))
        self.dt = 1.0 / self.fps
        self.dry_run = dry_run
        self.sock = None
        self.addr = (s["host"], int(s["port"]))
        self.idx = 0
        self.t0 = None
        if not dry_run:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.sendto(MAGIC + bytes([TYPE_META]) +
                             json.dumps({"fps": self.fps}).encode(), self.addr)

    def push(self, frames):
        if frames is None or len(frames) == 0:
            return 0
        arr = reorder(np.asarray(frames, dtype=np.float32))
        if self.t0 is None:
            self.t0 = time.perf_counter()
        for f in arr:
            target = self.t0 + self.idx * self.dt
            wait = target - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            if self.sock:
                self.sock.sendto(frame_packet(f, self.idx, time.time()), self.addr)
            self.idx += 1
        return len(arr)

    def finish(self):
        if self.sock:
            self.sock.sendto(MAGIC + bytes([TYPE_STOP]), self.addr)
            self.sock.close()
            self.sock = None
        return self.idx


def selftest():
    from a2f_infer import MODEL_BS_LIST, load_config
    assert len(ARKIT_NAMES) == 52
    assert set(ARKIT_NAMES) == set(MODEL_BS_LIST)
    bs = np.zeros((3, 52), dtype=np.float32)
    bs[:, MODEL_BS_LIST.index("jawOpen")] = 0.9
    r = reorder(bs)
    assert r.shape == (3, 52)
    assert abs(r[0, ARKIT_NAMES.index("jawOpen")] - 0.9) < 1e-6
    assert np.count_nonzero(r[0]) == 1, "重排把值放错了列"
    pkt = frame_packet(r[0], 7, 1.5)
    assert len(pkt) == HEADER_SIZE + 52 * 4
    magic, typ, idx, ts, count = struct.unpack(HEADER_FMT, pkt[:HEADER_SIZE])
    assert magic == MAGIC and typ == TYPE_FRAME and idx == 7 and count == 52
    vals = struct.unpack("!52f", pkt[HEADER_SIZE:])
    assert abs(vals[ARKIT_NAMES.index("jawOpen")] - 0.9) < 1e-6
    cfg = load_config()
    assert cfg["stream"]["port"] == 9876
    print("STREAM_SELFTEST_OK header=%d bytes, reorder ok, packet ok" % HEADER_SIZE)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npy", nargs="?")
    ap.add_argument("--config")
    ap.add_argument("--dry-run", action="store_true", help="只走节拍不发包")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.npy:
        ap.error("需要 .npy 路径，或用 --selftest")
    data = np.load(args.npy)
    n = send_stream(data, load_config(args.config), args.dry_run)
    print("STREAM_OK frames=%d dry_run=%s" % (n, args.dry_run))


if __name__ == "__main__":
    main()
