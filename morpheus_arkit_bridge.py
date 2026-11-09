"""单一 33 骨架的 ARKit 直连映射与 PWM 输出。

架构（46 根原骨删除后，没有 Targets -> 原骨 -> 二级骨这条链了）：

    ARKit 52 系数
       ├─> SampleHead / eyes 形态键（面部形变，保留原机制）
       ├─> servo_map.ARKIT_TO_SERVO  ->  33 根 Morpheus_Servo_XX 的旋转
       └─> servo_map.frame_to_pwm    ->  scene["morpheus_pwm"] 33 路硬件命令

用法：在 Blender 文本编辑器里 Run Script 一次，之后播放时间轴或推流 UDP
都会自动刷新；也可在脚本里直接调用 apply_frame() 取当前帧结果。

两个刷新通道：
  * frame_change_post —— 播放/拖动时间轴
  * 1/60s timer       —— UDP 推流直接写形态键时不触发 frame_change
"""

import bpy
import json
import math
import pathlib
import sys

ARMATURE_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"
SHAPE_OBJECT = "SampleHead"

# 骨骼运动由 mechanism_kinematics 按 CAD 真实几何正向驱动（无 IK/无求解器）。
# 输入 = 标准映射归一化值 -> frame_to_pwm -> 舵机转角。不再用占位 MAX_ANGLE。


def _root():
    p = pathlib.Path(bpy.data.filepath).resolve().parent
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
    return p


def _servo_map():
    _root()
    import servo_map
    return servo_map


def shape_key_blocks():
    ob = bpy.data.objects.get(SHAPE_OBJECT)
    if ob is None or ob.data.shape_keys is None:
        return None
    return ob.data.shape_keys.key_blocks


def read_arkit():
    """按 servo_map.MODEL_BS_LIST（render.py 的 52 维列序）读形态键值。"""
    sm = _servo_map()
    ks = shape_key_blocks()
    if ks is None:
        return [0.0] * len(sm.MODEL_BS_LIST)
    return [float(ks[n].value) if n in ks else 0.0 for n in sm.MODEL_BS_LIST]


def activation(values):
    """52 维 ARKit -> 33 路归一化激活值 [-1, 1]。"""
    sm = _servo_map()
    accum = {}
    for i, name in enumerate(sm.MODEL_BS_LIST):
        w = values[i]
        if abs(w) < 1e-7:
            continue
        for sid, coeff in sm.ARKIT_TO_SERVO.get(name, {}).items():
            accum[sid] = accum.get(sid, 0.0) + w * coeff
    return {sid: max(-1.0, min(1.0, a)) for sid, a in accum.items()}


_pwm_sock = [None]
_pwm_cfg = [None]
_specs = {}


def _spec(sid):
    if sid not in _specs:
        _root()
        import mechanism_kinematics as mk
        _specs[sid] = mk.build_spec(sid)
    return _specs[sid]


def _mk():
    _root()
    import mechanism_kinematics as mk
    return mk


def mk_servo_dtheta(sid, pwm_value):
    return _mk().servo_dtheta(sid, pwm_value)


def mk_pose(spec, dtheta, rest_matrix):
    return _mk().pose_matrix(spec, dtheta, rest_matrix)


def _pwm_out_cfg():
    """Blender 算出的 33 路 PWM 往哪里发（只读一次配置文件）。"""
    if _pwm_cfg[0] is None:
        cfg = {"enable": False}
        try:
            p = _root() / "hardware_config.json"
            st = json.loads(p.read_text(encoding="utf-8")).get("streaming", {})
            cfg = {
                "enable": bool(st.get("pwm_from_blender", True)),
                "host": st.get("pwm_udp_host", "127.0.0.1"),
                "port": int(st.get("pwm_udp_port", 9878)),
            }
        except Exception as exc:
            print("morpheus_arkit_bridge pwm cfg:", exc)
        _pwm_cfg[0] = cfg
    return _pwm_cfg[0]


def emit_pwm(pwm):
    """把 PWM 推给外部下发进程。全流式链路里，PWM 的唯一来源就是这里。"""
    cfg = _pwm_out_cfg()
    if not cfg["enable"]:
        return
    if _pwm_sock[0] is None:
        import socket
        _pwm_sock[0] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        _pwm_sock[0].sendto(
            json.dumps({"pwm": [int(v) for v in pwm]}, separators=(",", ":")).encode(),
            (cfg["host"], cfg["port"]))
    except Exception as exc:
        print("morpheus_arkit_bridge pwm send:", exc)


def apply_frame():
    """读当前形态键 -> 写 33 根骨旋转 + 33 路 PWM。返回 PWM 列表。"""
    sm = _servo_map()
    arm = bpy.data.objects.get(ARMATURE_NAME)
    if arm is None:
        return None
    values = read_arkit()
    act = activation(values)
    pwm = sm.frame_to_pwm(values)

    for sid in range(1, 34):
        pb = arm.pose.bones.get(PREFIX + "%02d" % sid)
        if pb is None:
            continue
        spec = _spec(sid)
        dtheta = mk_servo_dtheta(sid, pwm[sid - 1])
        pb.matrix = mk_pose(spec, dtheta, arm.data.bones[pb.name].matrix_local)

    # 眼球：M07 俯仰 / M24 水平注视，各自绕球心转（eyes.L/R 原点=球心）。
    # 与骨同源：同一套归一化值 -> PWM -> 舵机转角 -> 正向输出角（绕 spec 真实轴）。
    mk = _mk()
    s7, s24 = _spec(7), _spec(24)
    ang7 = mk.output_angle(s7, mk_servo_dtheta(7, pwm[6]))
    ang24 = mk.output_angle(s24, mk_servo_dtheta(24, pwm[23]))
    rot = s7["axis"] * ang7 + s24["axis"] * ang24
    for en in ("eyes.L", "eyes.R"):
        ob = bpy.data.objects.get(en)
        if ob is not None:
            ob.rotation_euler = tuple(rot)

    scene = bpy.context.scene
    scene["morpheus_pwm"] = [int(v) for v in pwm]
    scene["morpheus_activation"] = [round(act.get(i, 0.0), 5) for i in range(1, 34)]
    scene["morpheus_arkit"] = [round(v, 5) for v in values]
    scene["morpheus_pwm_ticks"] = int(scene.get("morpheus_pwm_ticks", 0)) + 1
    emit_pwm(pwm)
    return pwm


def extract_frame(frame_number=None):
    if frame_number is not None:
        bpy.context.scene.frame_set(frame_number)
    return apply_frame()


def extract_all_frames(start=None, end=None):
    sc = bpy.context.scene
    a = sc.frame_start if start is None else start
    b = sc.frame_end if end is None else end
    out = {}
    for f in range(a, b + 1):
        sc.frame_set(f)
        out[f] = apply_frame()
    return out


def export_json(filepath, start=None, end=None):
    data = {"source": "morpheus_arkit_bridge", "frames": extract_all_frames(start, end)}
    pathlib.Path(filepath).write_text(
        json.dumps(data, ensure_ascii=False, indent=1), encoding="utf-8")
    return filepath


# --------------------------------------------------------------- handlers
def _frame_handler(scene, depsgraph=None):
    try:
        apply_frame()
    except Exception as exc:
        print("morpheus_arkit_bridge frame error:", exc)


def _signature():
    ks = shape_key_blocks()
    if ks is None:
        return ()
    return tuple(round(float(k.value), 5) for k in ks)


_last_sig = [None]


def _timer_tick():
    try:
        sig = _signature()
        if sig != _last_sig[0]:
            _last_sig[0] = sig
            apply_frame()
    except Exception as exc:
        print("morpheus_arkit_bridge timer error:", exc)
    return 0.016


def install_handlers():
    for h in list(bpy.app.handlers.frame_change_post):
        if getattr(h, "_morpheus_arkit", False):
            bpy.app.handlers.frame_change_post.remove(h)
    _frame_handler._morpheus_arkit = True
    bpy.app.handlers.frame_change_post.append(_frame_handler)
    if not bpy.app.timers.is_registered(_timer_tick):
        bpy.app.timers.register(_timer_tick)
    _last_sig[0] = _signature()
    return apply_frame()


def uninstall_handlers():
    for h in list(bpy.app.handlers.frame_change_post):
        if getattr(h, "_morpheus_arkit", False):
            bpy.app.handlers.frame_change_post.remove(h)
    if bpy.app.timers.is_registered(_timer_tick):
        bpy.app.timers.unregister(_timer_tick)


if __name__ == "__main__":
    pwm = install_handlers()
    print("MORPHEUS_ARKIT_BRIDGE_OK servos=%d pwm=%s" % (len(pwm), pwm))
