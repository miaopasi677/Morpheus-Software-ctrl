"""把 33 路 PWM 发到舵机驱动板。

现在 PWM 只停在 Blender 的 Scene 属性 / JSON 文件里，这一步是最后一段。
因为还不知道你用哪块驱动板，这里做成可插拔 backend + 一个配置文件：
  backend = dry_run  不接硬件也能全链路自测
  backend = serial   串口，命令格式写在 hardware_config.json 的 frame 里
  backend = udp      发给 ESP32 / 网络驱动板

安全：限幅到 Table V 区间、单帧限速、超时自动回中立、Ctrl+C 回中立。

单独自检：
    python servo_output.py --selftest
逐通道通电核对：
    python calibrate_channels.py
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


def load_config(path=None):
    p = pathlib.Path(path or (ROOT / "hardware_config.json"))
    return json.loads(p.read_text(encoding="utf-8"))


def clamp_pwm(values):
    out = []
    for i, v in enumerate(values, 1):
        s = sm.BY_ID[i]
        out.append(int(round(max(s["pwm_low"], min(s["pwm_high"], float(v))))))
    return out


def neutral_pwm():
    return [int(s["pwm_start"]) for s in sm.SERVOS]


def load_channel_map(cfg):
    """逻辑 ID -> 物理通道号（1-based）。默认 1:1。

    calibrate_channels.py 核对实物后会把真实对应写进这个文件。
    """
    p = ROOT / cfg["servo"].get("channel_map", "channel_map.json")
    if not p.exists():
        return {str(i): i for i in range(1, 34)}
    d = json.loads(p.read_text(encoding="utf-8"))
    rows = d.get("map", d) if isinstance(d, dict) else d
    return {str(i): int(rows.get(str(i), i)) for i in range(1, 34)}


class Backend:
    def open(self):
        pass

    def write(self, physical_values):
        raise NotImplementedError

    def close(self):
        pass


class DryRunBackend(Backend):
    def __init__(self):
        self.n = 0

    def write(self, values):
        self.n += 1
        if self.n == 1 or self.n % 50 == 0:
            print("  [dry_run] frame %d sample=%s" % (self.n, values[:6]), flush=True)


class SerialBackend(Backend):
    def __init__(self, cfg):
        self.cfg = cfg
        self.ser = None

    def open(self):
        import serial
        self.ser = serial.Serial(self.cfg["port"], self.cfg["baud"], timeout=0.2)
        print("  [serial] %s @ %d 已打开" % (self.cfg["port"], self.cfg["baud"]))

    def write(self, values):
        frame = self.cfg["frame"].replace("{values}", ",".join(str(v) for v in values))
        frame = frame.replace("\\n", "\n").replace("\\r", "\r")
        self.ser.write(frame.encode("ascii"))

    def close(self):
        if self.ser:
            self.ser.close()


class UdpBackend(Backend):
    def __init__(self, cfg):
        self.cfg = cfg
        self.sock = None

    def open(self):
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def write(self, values):
        payload = json.dumps({"pwm": values}, separators=(",", ":")).encode()
        self.sock.sendto(payload, (self.cfg["host"], int(self.cfg["port"])))

    def close(self):
        if self.sock:
            self.sock.close()


def make_backend(cfg):
    s = cfg["servo"]
    kind = s.get("backend", "dry_run")
    if kind == "serial":
        return SerialBackend(s["serial"])
    if kind == "udp":
        return UdpBackend(s["udp"])
    return DryRunBackend()


class ServoOutput:
    def __init__(self, cfg=None, backend=None):
        self.cfg = cfg or load_config()
        s = self.cfg["servo"]
        self.backend = backend or make_backend(self.cfg)
        self.chmap = load_channel_map(self.cfg)
        self.max_step = int(s.get("max_step", 200))
        self.timeout = float(s.get("neutral_on_timeout_s", 1.0))
        self.last = neutral_pwm()
        self.last_send = time.perf_counter()

    def open(self):
        self.backend.open()
        self.send(neutral_pwm(), force=True)
        return self

    def send(self, pwm33, force=False):
        vals = clamp_pwm(pwm33)
        if self.max_step and not force:
            vals = [int(self.last[i] + max(-self.max_step,
                                           min(self.max_step, vals[i] - self.last[i])))
                    for i in range(33)]
        # 逻辑 ID -> 物理通道
        physical = [0] * 33
        for i in range(1, 34):
            ch = self.chmap[str(i)]
            if 1 <= ch <= 33:
                physical[ch - 1] = vals[i - 1]
        self.backend.write(physical)
        self.last = vals
        self.last_send = time.perf_counter()
        return vals

    def watchdog(self):
        """主循环卡住时自动回中立，防止舵机长期顶死。"""
        if time.perf_counter() - self.last_send > self.timeout:
            self.send(neutral_pwm(), force=True)
            return True
        return False

    def close(self):
        try:
            self.send(neutral_pwm(), force=True)
        finally:
            self.backend.close()


def selftest():
    cfg = load_config()
    out = ServoOutput(cfg, DryRunBackend())
    assert len(neutral_pwm()) == 33
    assert len(out.chmap) == 33
    v = clamp_pwm([99999] * 33)
    assert all(sm.BY_ID[i]["pwm_low"] <= v[i - 1] <= sm.BY_ID[i]["pwm_high"]
               for i in range(1, 34))
    # 限速：从 1000 一帧跳到 2000，最多走 max_step
    out.last = [1000] * 33
    stepped = out.send([2000] * 33)
    assert all(abs(a - 1000) <= out.max_step for a in stepped), stepped[:3]
    # 通道映射：交换 1/2 后物理通道 1 应拿到逻辑 2 的值
    out.chmap["1"], out.chmap["2"] = 2, 1
    captured = {}

    class Cap(DryRunBackend):
        def write(self, values):
            captured["v"] = list(values)

    out.backend = Cap()
    # 用各通道自己的 Table V 中立值做基准，避免 clamp 改掉测试值
    base = neutral_pwm()
    out.last = list(base)
    test = list(base)
    s2 = sm.BY_ID[2]
    target = s2["pwm_low"] + (1 if s2["pwm_start"] - s2["pwm_low"] >= 1 else 0)
    test[1] = target
    out.send(test, force=True)
    # 逻辑 2 -> 物理通道 1，所以 physical[0] 应等于 target
    assert captured["v"][0] == target, (captured["v"][:3], target)
    print("SERVO_SELFTEST_OK clamp/limit/channel_map/neutral all pass")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    cfg = load_config(args.config)
    out = ServoOutput(cfg).open()
    print("SERVO_OUTPUT backend=%s，Ctrl+C 退出并回中立" % cfg["servo"]["backend"])
    try:
        while True:
            out.watchdog()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        print("已回中立")


if __name__ == "__main__":
    main()
