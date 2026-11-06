"""
把 demo.py 输出的 52 维 blendshape .npy 转换成 Morpheus 硬件的 33 路 PWM 命令。

  python servo_pwm.py result/xxx.npy --out pwm.json
  python servo_pwm.py --selftest

链路：demo.py -> (T,52) npy -> ARKIT_TO_SERVO 种子映射 -> 26 维逆解通道
      -> norm_to_pwm() 按 Table V 的 Lower/Start/Upper 反归一化 -> 33 路 PWM
      -> 串口 / PCA9685 / Jetson 下发（下发部分按你的控制板自行接入）

重要：
  * ARKIT_TO_SERVO 是手工分组初值，不是论文训练出的 W。论文用 self-modeling MLP
    + 梯度反传求 W；上线前必须用实物采 5000 组数据重新标定，否则只是"能动"。
  * 每路 PWM 已 clamp 在各自允许区间内，未参与逆解的 7 路保持 Start Value。
  * 虚拟表情 30 fps、舵机 50 Hz，需要插值/保持（--servo-fps）。
"""

import argparse
import json
import pathlib
import re
import sys

try:
    import numpy as np
except ImportError:  # 系统 Python 可能没装 numpy，仍需能跑
    np = None

from servo_map import (MODEL_BS_LIST, SERVOS, BY_ID, INVERSE_IDS,
                       frame_to_pwm, norm_to_pwm)

# .blend 内 blender_receiver.py 的 ARKIT_NAMES 顺序（标准 ARKit，与 render.py 的
# model_bsList 顺序不同）。若数据来自 UDP 接收端，用 --order receiver 自动重排。
RECEIVER_ORDER = [
    "eyeBlinkLeft", "eyeLookDownLeft", "eyeLookInLeft", "eyeLookOutLeft",
    "eyeLookUpLeft", "eyeSquintLeft", "eyeWideLeft", "eyeBlinkRight",
    "eyeLookDownRight", "eyeLookInRight", "eyeLookOutRight", "eyeLookUpRight",
    "eyeSquintRight", "eyeWideRight", "jawForward", "jawLeft", "jawRight",
    "jawOpen", "mouthClose", "mouthFunnel", "mouthPucker", "mouthLeft",
    "mouthRight", "mouthSmileLeft", "mouthSmileRight", "mouthFrownLeft",
    "mouthFrownRight", "mouthDimpleLeft", "mouthDimpleRight",
    "mouthStretchLeft", "mouthStretchRight", "mouthRollLower", "mouthRollUpper",
    "mouthShrugLower", "mouthShrugUpper", "mouthPressLeft", "mouthPressRight",
    "mouthLowerDownLeft", "mouthLowerDownRight", "mouthUpperUpLeft",
    "mouthUpperUpRight", "browDownLeft", "browDownRight", "browInnerUp",
    "browOuterUpLeft", "browOuterUpRight", "cheekPuff", "cheekSquintLeft",
    "cheekSquintRight", "noseSneerLeft", "noseSneerRight", "tongueOut",
]


def render_order_from_file(path=None):
    """从 render.py 解析 model_bsList，确认 servo_map.MODEL_BS_LIST 没写错。"""
    p = pathlib.Path(path or pathlib.Path(__file__).with_name("render.py"))
    txt = p.read_text(encoding="utf-8")
    m = re.search(r"model_bsList\s*=\s*\[(.*?)\]", txt, re.S)
    if not m:
        return []
    return re.findall(r'"([^"]+)"', m.group(1))


def reorder(vector, order):
    """把 order 顺序的 52 维向量重排成 render.py 的 MODEL_BS_LIST 顺序。"""
    idx = [order.index(n) for n in MODEL_BS_LIST]
    return [vector[i] for i in idx]


def load_npy(path):
    """读 (T,52) .npy；无 numpy 时用标准库解析 float32/float64 的二维 C 序数组。"""
    if np is not None:
        return np.load(path).tolist()
    import ast
    import struct
    with open(path, "rb") as f:
        if f.read(6) != b"\x93NUMPY":
            raise ValueError("不是 .npy 文件")
        major = f.read(2)[0]  # major, minor 两个版本字节都要读掉
        hlen = struct.unpack("<H" if major == 1 else "<I",
                             f.read(2 if major == 1 else 4))[0]
        head = ast.literal_eval(f.read(hlen).decode("latin1")
                                .replace("\x00", "").strip())
        dtype, shape = head["descr"], head["shape"]
        if head["fortran_order"] or len(shape) != 2:
            raise ValueError("仅支持二维 C 序数组")
        code = {"<f4": "f", "<f8": "d", "<f2": "e", "|f4": "f", "|f8": "d"}.get(dtype)
        if code is None:
            raise ValueError("不支持的 dtype: %s" % dtype)
        n = shape[0] * shape[1]
        vals = struct.unpack("<" + code * n, f.read(struct.calcsize("<" + code) * n))
    return [list(vals[i * shape[1]:(i + 1) * shape[1]]) for i in range(shape[0])]


def convert(bs, order="render", smooth=0, servo_fps=50, source_fps=30,
            max_step=None):
    """(T,52) -> (T,33) PWM，返回 list[list[int]]（不依赖 numpy）。"""
    src = RECEIVER_ORDER if order == "receiver" else MODEL_BS_LIST
    rows = []
    for row in bs:
        v = reorder(row, src) if order == "receiver" else list(row)
        rows.append([float(x) for x in frame_to_pwm(v)])

    if smooth and smooth > 1 and len(rows) > smooth:
        w = smooth
        rows = [[sum(r[i] for r in rows[max(0, t - w // 2):min(len(rows), t + w // 2 + 1)]) /
                 len(rows[max(0, t - w // 2):min(len(rows), t + w // 2 + 1)])
                 for i in range(33)] for t in range(len(rows))]

    for r in rows:
        for i, s in enumerate(SERVOS):
            r[i] = min(s["pwm_high"], max(s["pwm_low"], r[i]))

    # 虚拟 30fps -> 舵机 50Hz：线性插值
    if servo_fps and servo_fps != source_fps and len(rows) > 1:
        n = int(round((len(rows) - 1) * servo_fps / source_fps)) + 1
        out = []
        for k in range(n):
            x = k * (len(rows) - 1) / max(1, n - 1)
            i0, f = int(x), x - int(x)
            i1 = min(len(rows) - 1, i0 + 1)
            out.append([rows[i0][j] * (1 - f) + rows[i1][j] * f for j in range(33)])
        rows = out

    if max_step:
        for t in range(1, len(rows)):
            rows[t] = [rows[t - 1][j] + max(-max_step, min(max_step,
                       rows[t][j] - rows[t - 1][j])) for j in range(33)]

    return [[int(round(v)) for v in r] for r in rows]


def selftest():
    print("servos=%d inverse=%d" % (len(SERVOS), len(INVERSE_IDS)))
    order = render_order_from_file()
    print("render.py model_bsList == servo_map.MODEL_BS_LIST :", order == MODEL_BS_LIST)
    print("receiver ARKIT_NAMES 顺序与 render.py 相同 :", RECEIVER_ORDER == MODEL_BS_LIST)
    if order != MODEL_BS_LIST:
        print("  !! 顺序不一致，请核对 servo_map.MODEL_BS_LIST")

    neutral = frame_to_pwm([0.0] * 52)
    starts = [s["pwm_start"] for s in SERVOS]
    print("全 0 输入 -> 33 路 Start Value :", neutral == starts)
    if neutral != starts:
        diff = [(i + 1, a, b) for i, (a, b) in enumerate(zip(neutral, starts)) if a != b]
        print("  偏差:", diff)

    for sid in (1, 3, 23, 30, 33):
        s = BY_ID[sid]
        vals = [norm_to_pwm(sid, v) for v in (0.0, 0.5, 1.0)]
        print("ID %2d %-28s low=%4d start=%4d high=%4d dir=%+d  norm 0/.5/1 -> %s" %
              (sid, s["name"], s["pwm_low"], s["pwm_start"], s["pwm_high"],
               s["direction"], vals))


def main():
    ap = argparse.ArgumentParser(description="52 维 blendshape -> 33 路 Morpheus 舵机 PWM")
    ap.add_argument("npy", nargs="?", help="demo.py 输出的 .npy (T,52)")
    ap.add_argument("--out", help="输出 json / csv 路径")
    ap.add_argument("--order", choices=["render", "receiver"], default="render")
    ap.add_argument("--smooth", type=int, default=0, help="savgol 窗口，0=不平滑")
    ap.add_argument("--servo-fps", type=int, default=50)
    ap.add_argument("--source-fps", type=int, default=30)
    ap.add_argument("--max-step", type=int, default=None, help="单帧最大 PWM 变化")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest or not a.npy:
        selftest()
        if not a.npy:
            return

    bs = load_npy(a.npy)
    if not bs or len(bs[0]) != 52:
        raise ValueError("期望 (T,52)，实际 (%d,%d)" % (len(bs), len(bs[0]) if bs else 0))
    pwm = convert(bs, a.order, a.smooth, a.servo_fps, a.source_fps, a.max_step)
    print("input (%d,%d) -> pwm (%d,%d)  (fps %d)" %
          (len(bs), len(bs[0]), len(pwm), len(pwm[0]), a.servo_fps))

    if a.out:
        out = pathlib.Path(a.out)
        if out.suffix.lower() == ".csv":
            import csv
            with out.open("w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["frame"] + ["M%02d" % s["id"] for s in SERVOS])
                for t, row in enumerate(pwm):
                    w.writerow([t] + row)
        else:
            out.write_text(json.dumps({
                "source": "arxiv:2507.16645v1 Table V",
                "servo_fps": a.servo_fps,
                "note": "ARKIT_TO_SERVO 为手工初值，需实物标定后替换",
                "channels": ["M%02d_%s" % (s["id"], s["name"]) for s in SERVOS],
                "frames": pwm,
            }, ensure_ascii=False), encoding="utf-8")
        print("saved:", out)


if __name__ == "__main__":
    main()
