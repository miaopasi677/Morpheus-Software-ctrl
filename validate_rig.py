import bpy
import json
import math
import mathutils
import pathlib
import sys

ROOT = pathlib.Path(r"C:\file\project\Morpheus-Software")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import blender_servo_bridge as bridge
import servo_map as sm

original = json.loads((ROOT / "original_disk_blend.json").read_text(encoding="utf-8"))
arm = bpy.data.objects["Feace_Rig.001"]
mesh = bpy.data.objects["SampleHead"]
keys = mesh.data.shape_keys
assert len(arm.data.bones) == 46
assert len(original["bones"]) == 46
assert bpy.data.objects.get("Morpheus_Servo_Rig") is None
assert not any(b.name.startswith("Servo_") for b in arm.data.bones)

# 原文件的几何和已保存标定必须逐项一致。
for name, old in original["bones"].items():
    new_bone = arm.data.bones[name]
    assert list(new_bone.head_local) == old["head_local"], name + " head differs"
    assert list(new_bone.tail_local) == old["tail_local"], name + " tail differs"
    old_settings = old["props"].get("servo_settings", {})
    new_settings = new_bone.get("servo_settings", {})
    for key, value in old_settings.items():
        assert key in new_settings and new_settings[key] == value, (name, key, value, new_settings.get(key))


def original_matrix_visual(pose_bone):
    bone = pose_bone.bone
    matrix_pose_bone = pose_bone.matrix
    matrix_bone = bone.matrix_local
    if bone.parent:
        matrix_parent_bone = bone.parent.matrix_local.copy()
        matrix_parent_pose_bone = pose_bone.parent.matrix.copy()
    else:
        matrix_parent_bone = mathutils.Matrix()
        matrix_parent_pose_bone = mathutils.Matrix()
    matrix_parent_pose_bone_inverted = matrix_parent_pose_bone.copy().inverted()
    return (
        matrix_bone.copy().inverted()
        @ matrix_parent_bone
        @ matrix_parent_pose_bone_inverted
        @ matrix_pose_bone
    )


def original_position(bone_name):
    pose_bone = arm.pose.bones[bone_name]
    settings = dict(bridge.DEFAULTS)
    settings.update(original["bones"][bone_name]["props"].get("servo_settings", {}))
    rotation_euler = original_matrix_visual(pose_bone).to_euler()
    degrees = round(math.degrees(rotation_euler[int(settings["rotation_axis"])]) *
                    float(settings["multiplier"]), 2)
    if settings["reverse_direction"]:
        degrees *= -1
    angle = float(settings["neutral_angle"]) - degrees
    return round(bridge.range_map(
        angle, 0, float(settings["rotation_range"]),
        float(settings["position_min"]), float(settings["position_max"])))


def check_pwm(rows):
    for row in rows:
        low = min(row["lower"], row["upper"])
        high = max(row["lower"], row["upper"])
        assert low <= row["pwm"] <= high, row
        if not row["mapped"]:
            assert row["pwm"] == row["start"], row


# A. 对原激活动作逐帧比较旧算法和新读取方式；不要求这段全 0 动作产生运动。
frames = [1, 5, 10, 15, 20, 25, 30]
legacy_comparisons = []
original_action_frames = {}
original_key_max = 0.0
for frame in frames:
    bpy.context.scene.frame_set(frame)
    original_key_max = max(original_key_max, max(abs(k.value) for k in keys.key_blocks))
    expected = [original_position(name) for name in bridge.LEGACY_BONES]
    actual = bridge.extract_frame()
    assert actual == expected, (frame, actual, expected)
    legacy_comparisons.append({"frame": frame, "exact": True, "pwm": actual})
    rows = bridge.morpheus_details()
    check_pwm(rows)
    original_action_frames[frame] = [row["pwm"] for row in rows]

pwm_path = bridge.export_morpheus_json(ROOT / "morpheus_pwm.json")

# B. 暂时摘掉全 0 动作并注入非零 ARKit；后台验证后退出，不保存这个动作改动。
keys.animation_data.action = None
bpy.context.scene.frame_set(1)
for key in keys.key_blocks:
    key.value = 0.0
mesh.update_tag()
bpy.context.view_layer.update()

target_names = ("browInnerE.R", "browOuterE.L", "eyeE.L", "eyeLidUpE.R",
                "lipUpE.L", "lipDownE.R", "lipConnerE.L", "lipConnerE.R", "jawE")


def snapshot():
    mesh.update_tag()
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    targets = {}
    for name in target_names:
        obj = bpy.data.objects[name].evaluated_get(depsgraph)
        targets[name] = list(obj.matrix_world.translation)
    rows = bridge.morpheus_details()
    check_pwm(rows)
    return targets, rows


neutral_targets, neutral_rows = snapshot()
test_values = {
    "browInnerUp": 0.75,
    "browOuterUpLeft": 0.55,
    "eyeBlinkRight": 0.65,
    "eyeLookUpLeft": 0.45,
    "eyeLookInLeft": 0.35,
    "jawOpen": 0.80,
    "jawLeft": 0.35,
    "mouthSmileLeft": 0.75,
    "mouthSmileRight": 0.45,
    "mouthUpperUpLeft": 0.40,
    "mouthLowerDownRight": 0.50,
}
for name, value in test_values.items():
    keys.key_blocks[name].value = value
test_targets, test_rows = snapshot()

target_travel = {
    name: math.sqrt(sum((test_targets[name][i] - neutral_targets[name][i]) ** 2 for i in range(3)))
    for name in target_names
}
assert sum(1 for distance in target_travel.values() if distance > 1e-5) >= 4, target_travel

neutral_pwm = [row["pwm"] for row in neutral_rows]
test_pwm = [row["pwm"] for row in test_rows]
varying_channels = [sid for sid in range(1, 34) if neutral_pwm[sid - 1] != test_pwm[sid - 1]]
assert len(varying_channels) >= 8, varying_channels
# 单边左眼输入也必须进入双眼联动通道，Jaw_Open 必须向唯一可动端点运动。
assert test_pwm[6] != sm.BY_ID[7]["start"], (neutral_pwm[6], test_pwm[6])
assert test_pwm[23] != sm.BY_ID[24]["start"], (neutral_pwm[23], test_pwm[23])
assert test_pwm[22] != sm.BY_ID[23]["start"], (neutral_pwm[22], test_pwm[22])
assert sm.BY_ID[23]["start"] < test_pwm[22] <= sm.BY_ID[23]["lower"], test_pwm[22]

# 实时处理器必须在 frame_set 后更新 Scene 属性。
bridge.install_live_handler()
bpy.context.scene.frame_set(15)
assert list(bpy.context.scene["morpheus_pwm"]) == bridge.extract_morpheus_frame()
assert list(bpy.context.scene["legacy_pwm"]) == bridge.extract_frame()

report = {
    "status": "PASS",
    "original_file": original["file"],
    "validated_file": bpy.data.filepath,
    "bones": len(arm.data.bones),
    "placeholder_bones": 0,
    "parallel_servo_armature": False,
    "original_geometry_and_servo_settings_exact": True,
    "legacy_29_exact_frames": legacy_comparisons,
    "original_action_max_abs_shape_key": original_key_max,
    "original_action_note": "当前激活动作所有采样帧形态键均为 0；非零 ARKit 另行注入验证",
    "original_action_morpheus_frames": original_action_frames,
    "morpheus_bounds_and_neutral_checks": "PASS",
    "mapped_channels": sum(1 for s in sm.SERVOS if s["bones"]),
    "neutral_only_channels": [s["id"] for s in sm.SERVOS if not s["bones"]],
    "injected_arkit": test_values,
    "neutral_morpheus_pwm": neutral_pwm,
    "test_morpheus_pwm": test_pwm,
    "varying_morpheus_channels": varying_channels,
    "target_max_travel_blender_units": target_travel,
    "linked_eye_pitch_and_yaw_move": True,
    "jaw_open_moves_to_available_endpoint": True,
    "live_scene_properties": True,
    "pwm_export": str(pwm_path),
}
(ROOT / "rig_validation_report.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
print("RIG_VALIDATION_PASS", json.dumps({
    "bones": 46,
    "legacy_exact_frames": frames,
    "original_action_max_abs_shape_key": original_key_max,
    "varying_morpheus_channels": varying_channels,
    "target_travel": target_travel,
    "jaw_open_pwm": test_pwm[22],
    "eye_pitch_pwm": test_pwm[6],
    "eye_yaw_pwm": test_pwm[23],
    "pwm_export": str(pwm_path),
}, ensure_ascii=False))
