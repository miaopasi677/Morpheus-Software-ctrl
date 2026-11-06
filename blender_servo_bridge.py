"""
原 Feace_Rig.001 骨骼 -> 原 29 路 PWM / Morpheus 33 路 PWM。

核心原则：
1. ARKit -> SampleHead 形态键 -> 顶点父级 Targets -> 原骨骼约束/驱动器，全部保持不变。
2. 使用每根原骨骼自己的 servo_settings（轴、倍率、反向、中立角、角度范围）。
3. 不移动骨骼，不创建 Servo_* 骨骼，不注册/覆盖 servo_settings PropertyGroup。
4. 原 extract_frame() 保留原 29 路顺序；extract_morpheus_frame() 额外输出 33 路。
5. 没有原骨骼来源的 Morpheus 通道保持 Table V Start Value，不伪造运动。

在 Blender 文本编辑器运行后，播放动画时 Scene['morpheus_pwm'] 会逐帧更新。
导出当前动画：blender_servo_bridge.export_morpheus_json('C:/.../morpheus_pwm.json')
"""

import bpy
import json
import math
import mathutils
import pathlib
import sys

ARMATURE_NAME = "Feace_Rig.001"

# 与原文件内 get_position 的 bones_list 完全一致，不能按 servo_id 重排。
LEGACY_BONES = [
    "Brow_Out_Rotation.r", "Brow_Out_Rotation.l",
    "Brow_In_Rotation_pitch.r", "Brow_In_Rotation_pitch.l",
    "Eyelid_Up.r", "Eyelid_Up.l", "Eyelid_Dwn.r", "Eyelid_Dwn.l",
    "Eye_yaw.r", "Eye_yaw.l", "Eye_pitch.r", "Eye_pitch.l",
    "Lip_Up_Rotation.r", "Lip_Up_Rotation_C", "Lip_Up_Out_C", "Lip_Up_Rotation.l",
    "Lip_Dwn_Rotation.r", "Lip_Dwn_Rotation_C", "Lip_Dwn_Out_C", "Lip_Dwn_Rotation.l",
    "Smile_Rotation.r", "Lip_Corner_Rotation.r",
    "Smile_Rotation.l", "Lip_Corner_Rotation.l",
    "Jaw_Open", "Jaw_Yaw", "Neck_1", "Neck_2", "Neck_3",
]

# 与旧版脚本变量名兼容。
bones_list = LEGACY_BONES

# 原插件属性的实际默认值。缺失字段必须按这些值读，不能统一假设 Z 轴。
DEFAULTS = {
    "position_min": 500,
    "position_max": 2500,
    "neutral_angle": 90.0,
    "rotation_range": 180.0,
    "rotation_axis": 0,
    "multiplier": 1.0,
    "reverse_direction": False,
    "threshold": 2000,
    "servo_id": 0,
}


def _load_servo_map():
    root = pathlib.Path(bpy.data.filepath).resolve().parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    import servo_map
    return servo_map


def get_settings(bone):
    """优先读 .blend 已保存的 IDProperty；不依赖已失效的原插件注册。"""
    saved = bone.get("servo_settings")
    result = dict(DEFAULTS)
    if saved:
        for key in DEFAULTS:
            if key in saved:
                result[key] = saved[key]
        for key in ("active",):
            if key in saved:
                result[key] = saved[key]
    else:
        # 仅兼容仍安装原插件的会话；正常情况下走上面的已保存字典。
        try:
            runtime = bone.servo_settings
            for key in DEFAULTS:
                if hasattr(runtime, key):
                    result[key] = getattr(runtime, key)
        except (AttributeError, TypeError):
            pass
    result["rotation_axis"] = int(result["rotation_axis"])
    result["reverse_direction"] = bool(result["reverse_direction"])
    return result


def matrix_visual(pose_bone):
    """原 get_position.py 的骨架局部姿态矩阵算法，逐字保持同一数学链。"""
    bone = pose_bone.bone
    matrix_pose_bone = pose_bone.matrix
    matrix_bone = bone.matrix_local
    if bone.parent:
        matrix_parent_bone = bone.parent.matrix_local.copy()
        matrix_parent_pose_bone = pose_bone.parent.matrix.copy()
    else:
        matrix_parent_bone = mathutils.Matrix()
        matrix_parent_pose_bone = mathutils.Matrix()
    matrix_bone_inverted = matrix_bone.copy().inverted()
    matrix_parent_pose_bone_inverted = matrix_parent_pose_bone.copy().inverted()
    return (
        matrix_bone_inverted
        @ matrix_parent_bone
        @ matrix_parent_pose_bone_inverted
        @ matrix_pose_bone
    )


def range_map(value, from_low, from_high, to_low, to_high):
    if from_high == from_low:
        raise ValueError("角度范围不能为 0")
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


def _armature():
    arm = bpy.data.objects.get(ARMATURE_NAME)
    if arm is None or arm.type != "ARMATURE":
        raise RuntimeError("找不到原骨架 " + ARMATURE_NAME)
    return arm


def legacy_details(bone_name):
    pose_bone = _armature().pose.bones[bone_name]
    settings = get_settings(pose_bone.bone)
    rotation_euler = matrix_visual(pose_bone).to_euler()
    axis = settings["rotation_axis"]
    rotation_degrees = round(
        math.degrees(rotation_euler[axis]) * float(settings["multiplier"]), 2)
    if settings["reverse_direction"]:
        rotation_degrees *= -1
    angle = float(settings["neutral_angle"]) - rotation_degrees
    position = round(range_map(
        angle, 0.0, float(settings["rotation_range"]),
        float(settings["position_min"]), float(settings["position_max"])))
    return {
        "bone": bone_name,
        "legacy_servo_id": int(settings["servo_id"]),
        "axis": axis,
        "multiplier": float(settings["multiplier"]),
        "reverse": settings["reverse_direction"],
        "rotation_degrees": rotation_degrees,
        "angle": angle,
        "position": int(position),
        "settings": settings,
    }


def extract_frame(frame_number=None):
    """原 29 路输出；数学、顺序、取整与原 get_position.py 一致。"""
    if frame_number is not None:
        bpy.context.scene.frame_set(int(frame_number))
    return [legacy_details(name)["position"] for name in LEGACY_BONES]


def _remap_to_morpheus(legacy_position, settings, servo):
    """保留原骨骼标定曲线，以其原中立点为锚，分段映射到 Table V 三值。

    Table V 的 lower/start/upper 是命令端点，不要求 lower < upper。
    原骨骼静止时严格得到 start；向原 Pulse 两端运动时分别到 lower/upper。
    """
    old_min = float(settings["position_min"])
    old_max = float(settings["position_max"])
    if old_max == old_min:
        return int(servo["start"])
    q = (float(legacy_position) - old_min) / (old_max - old_min)
    q = max(0.0, min(1.0, q))
    neutral_position = range_map(
        float(settings["neutral_angle"]), 0.0, float(settings["rotation_range"]),
        old_min, old_max)
    q_neutral = max(0.0, min(1.0, (neutral_position - old_min) / (old_max - old_min)))
    lower, start, upper = float(servo["lower"]), float(servo["start"]), float(servo["upper"])
    # Jaw_Open 等通道 start == upper，唯一可动方向是 lower；不能把正向运动
    # 映回同一个 start，否则张颌会永远静止。
    positive_endpoint = upper if upper != start else lower
    negative_endpoint = lower if positive_endpoint == upper else upper
    if q >= q_neutral:
        t = (q - q_neutral) / (1.0 - q_neutral) if q_neutral < 1.0 else 0.0
        value = start + t * (positive_endpoint - start)
    else:
        t = q / q_neutral if q_neutral > 0.0 else 1.0
        value = negative_endpoint + t * (start - negative_endpoint)
    value = max(min(lower, upper), min(max(lower, upper), value))
    return int(round(value))


def morpheus_details(frame_number=None):
    if frame_number is not None:
        bpy.context.scene.frame_set(int(frame_number))
    sm = _load_servo_map()
    cache = {name: legacy_details(name) for name in LEGACY_BONES}
    rows = []
    for servo in sm.SERVOS:
        sources = [name for name in servo["bones"] if name in cache]
        if sources:
            weights = getattr(sm, "SOURCE_WEIGHTS", {}).get(
                servo["id"], {sources[0]: 1.0})
            mapped_positions = {}
            for source_name in sources:
                detail = cache[source_name]
                mapped_positions[source_name] = _remap_to_morpheus(
                    detail["position"], detail["settings"], servo)
            pwm_float = float(servo["start"]) + sum(
                float(weights.get(source_name, 0.0)) * (mapped_positions[source_name] - servo["start"])
                for source_name in sources)
            low, high = min(servo["lower"], servo["upper"]), max(servo["lower"], servo["upper"])
            pwm = int(round(max(low, min(high, pwm_float))))
            source = list(sources)
            legacy_position = {name: cache[name]["position"] for name in sources}
        else:
            source = []
            legacy_position = None
            pwm = int(servo["start"])
        rows.append({
            "id": servo["id"], "name": servo["name"], "source_bone": source,
            "legacy_position": legacy_position, "pwm": pwm,
            "lower": servo["lower"], "start": servo["start"], "upper": servo["upper"],
            "mapped": bool(source),
        })
    return rows


def extract_morpheus_frame(frame_number=None):
    return [row["pwm"] for row in morpheus_details(frame_number)]


def extract_all_frames():
    scene = bpy.context.scene
    return [extract_frame(frame) for frame in range(scene.frame_start, scene.frame_end + 1)]


def extract_morpheus_all_frames():
    scene = bpy.context.scene
    return [extract_morpheus_frame(frame) for frame in range(scene.frame_start, scene.frame_end + 1)]


def export_morpheus_json(filepath, start=None, end=None):
    scene = bpy.context.scene
    start = scene.frame_start if start is None else int(start)
    end = scene.frame_end if end is None else int(end)
    sm = _load_servo_map()
    frames = []
    previous = scene.frame_current
    try:
        for frame in range(start, end + 1):
            frames.append(extract_morpheus_frame(frame))
    finally:
        scene.frame_set(previous)
    payload = {
        "source": "original Feace_Rig.001 transforms + original per-bone calibration + Table V bounds",
        "frame_start": start,
        "frame_end": end,
        "fps": scene.render.fps / scene.render.fps_base,
        "channels": ["M%02d_%s" % (s["id"], s["name"]) for s in sm.SERVOS],
        "neutral_only_channels": [s["id"] for s in sm.SERVOS if not s["bones"]],
        "frames": frames,
    }
    path = pathlib.Path(filepath)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    return path


def _live_frame_handler(scene, depsgraph=None):
    rows = morpheus_details()
    scene["morpheus_pwm"] = [int(row["pwm"]) for row in rows]
    scene["morpheus_sources"] = json.dumps(
        {row["id"]: row["source_bone"] for row in rows}, ensure_ascii=False)
    scene["legacy_pwm"] = [int(legacy_details(name)["position"]) for name in LEGACY_BONES]
    scene["morpheus_pwm_frame"] = int(scene.frame_current)
    scene["morpheus_pwm_ticks"] = int(scene.get("morpheus_pwm_ticks", 0)) + 1


def install_live_handler():
    """幂等安装；本次 Blender 会话内播放/逐帧时更新 Scene 自定义属性。"""
    for handler in list(bpy.app.handlers.frame_change_post):
        if getattr(handler, "_morpheus_original_bone_bridge", False):
            bpy.app.handlers.frame_change_post.remove(handler)
    _live_frame_handler._morpheus_original_bone_bridge = True
    bpy.app.handlers.frame_change_post.append(_live_frame_handler)
    _live_frame_handler(bpy.context.scene)
    return len(bpy.app.handlers.frame_change_post)


# --- 实时 ARKit（UDP / 手动改形态键）--------------------------------------
# 原 blender_receiver.py 用 modal timer 直接写 shape key，不触发 frame_change，
# 所以只装 frame_change_post 不够。这里用轻量 timer 监听形态键变化。
_realtime_state = {
    "registered": False,
    "targets": None,
    "last_signature": None,
    "updates": 0,
}


def _shape_key_targets():
    cached = _realtime_state.get("targets")
    if cached:
        alive = []
        for obj in cached:
            try:
                if obj.name in bpy.data.objects and obj.data.shape_keys:
                    alive.append(obj)
            except ReferenceError:
                continue
        if alive:
            return alive
    targets = [o for o in bpy.data.objects if o.type == "MESH" and o.data.shape_keys]
    _realtime_state["targets"] = targets
    return targets


def _shape_key_signature():
    values = []
    for obj in _shape_key_targets():
        for key_block in obj.data.shape_keys.key_blocks:
            values.append(round(float(key_block.value), 4))
    return tuple(values)


def _realtime_tick():
    """形态键一变就重算；未变化时几乎零开销。返回下次间隔。"""
    try:
        signature = _shape_key_signature()
        if signature != _realtime_state["last_signature"]:
            _realtime_state["last_signature"] = signature
            bpy.context.view_layer.update()
            _live_frame_handler(bpy.context.scene)
            _realtime_state["updates"] += 1
    except Exception as exc:  # 不能让 timer 异常中断用户操作
        print("[Morpheus] realtime update failed:", exc)
    return 1.0 / 60.0


def install_realtime_handler():
    """幂等安装实时 timer；UDP 推流或手动拖形态键时同步更新 PWM。"""
    if bpy.app.timers.is_registered(_realtime_tick):
        bpy.app.timers.unregister(_realtime_tick)
    _realtime_state["registered"] = True
    _realtime_state["last_signature"] = None
    bpy.app.timers.register(_realtime_tick, first_interval=0.05, persistent=True)
    _realtime_tick()
    return _realtime_state["registered"]


def uninstall_realtime_handler():
    if bpy.app.timers.is_registered(_realtime_tick):
        bpy.app.timers.unregister(_realtime_tick)
    _realtime_state["registered"] = False
    return True


# 无论以文本块运行、被 import，还是被设为 use_module 自动加载，都幂等安装。
install_live_handler()
install_realtime_handler()
print("MORPHEUS_BRIDGE_OK original_bones=46 legacy=29 morpheus=33 realtime=on")
print("CURRENT_MORPHEUS_PWM", extract_morpheus_frame())
