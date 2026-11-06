"""硬件到位后第一件事：逐通道通电，把 33 路逻辑 ID 和实物舵机对上号。

现在 servo_id_mapping.json 是按 CAD 几何推断的，嘴部 16 路尤其不可靠。
这个脚本一次只让一个物理通道动，你看着实物回答它对应 Table V 的哪一路，
最后生成 channel_map.json 给 servo_output.py 用。

    python calibrate_channels.py                 # 真下发（按 hardware_config.json 的 backend）
    python calibrate_channels.py --dry-run       # 只打印不发包，先熟悉流程
    python calibrate_channels.py --selftest      # 不交互，自检

安全：每次只动一路，幅度限制在 Table V 区间内，退出自动回中立。
"""

import argparse
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm
from servo_output import load_config, neutral_pwm, clamp_pwm, make_backend


def build_map(cfg, dry_run=False, channel_range=None, answers=None):
    """answers: {物理通道: 逻辑ID}，给 selftest 用；None 则交互输入。"""
    backend = make_backend(cfg)
    backend.open()
    mapping = {str(i): i for i in range(1, 34)}
    notes = {}
    neutral = neutral_pwm()
    try:
        backend.write(neutral)
        chans = channel_range or range(1, 34)
        for ch in chans:
            # 只让这个物理通道偏离中立，其余保持 Table V 中立值
            low = list(neutral)
            high = list(neutral)
            s = sm.BY_ID[ch]
            low[ch - 1] = s["pwm_low"]
            high[ch - 1] = s["pwm_high"]
            print("\n--- 物理通道 %02d ---" % ch)
            print("  依次发送 %d -> %d -> %d（其余 32 路保持中立）" %
                  (s["pwm_low"], s["pwm_start"], s["pwm_high"]))
            if not dry_run:
                for v in (low, high, neutral):
                    backend.write(clamp_pwm(v))
                    time.sleep(0.6)
                backend.write(neutral)
            if answers is not None:
                mapping[str(answers.get(ch, ch))] = ch
                continue
            try:
                ans = input("  这个动作对应 Table V 的第几路？(回车=%d 同号, s=跳过, q=退出) " % ch).strip()
            except EOFError:
                ans = "s"
            if ans.lower() == "q":
                break
            if ans.lower() == "s" or ans == "":
                if ans == "":
                    mapping[str(ch)] = ch
                continue
            if ans.isdigit() and 1 <= int(ans) <= 33:
                mapping[str(int(ans))] = ch
                note = input("  备注（动了哪个部位，可回车跳过）: ").strip()
                if note:
                    notes[str(int(ans))] = note
            else:
                print("  输入无效，按同号处理")
    finally:
        backend.write(neutral)
        backend.close()
    return {"map": mapping, "notes": notes,
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "source": "calibrate_channels.py"}


def selftest():
    cfg = load_config()
    saved = cfg["servo"].get("backend")
    cfg["servo"]["backend"] = "dry_run"
    # 交换 1 和 2：物理通道 1 实际是逻辑 2，反之亦然
    answers = {1: 2, 2: 1}
    answers.update({i: i for i in range(3, 34)})
    d = build_map(cfg, dry_run=True, channel_range=range(1, 4), answers=answers)
    assert d["map"]["1"] == 2 and d["map"]["2"] == 1, d["map"]
    assert set(d["map"]) == {str(i) for i in range(1, 34)}
    cfg["servo"]["backend"] = saved
    print("CALIBRATE_SELFTEST_OK map swap ok, neutral restored")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out", default="channel_map.json")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    cfg = load_config(args.config)
    print("backend=%s  channel_map=%s" % (cfg["servo"]["backend"], args.out))
    print("确认已接好电源、舵机无卡阻、手远离连杆。Ctrl+C 随时回中立。\n")
    d = build_map(cfg, dry_run=args.dry_run)
    p = ROOT / args.out
    p.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
    changed = {k: v for k, v in d["map"].items() if int(k) != v}
    print("\nCALIBRATE_OK -> %s，与默认 1:1 不同的通道：%s" % (p, changed or "无"))


if __name__ == "__main__":
    main()
