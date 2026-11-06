"""
修复版：把 Morpheus 33 路输出接到原 Feace_Rig.001，不重建、不移动原骨骼。

用法：
  blender --background LY_GENERAL_V0.blend --python apply_servo_map.py
  blender --background LY_GENERAL_V0.blend --python apply_servo_map.py --python-expr "import bpy; bpy.ops.wm.save_mainfile()"

原控制链保持不动：
  ARKit -> SampleHead 形态键 -> 顶点父级 Targets -> 原骨骼约束/驱动器
  -> 原 servo_settings -> matrix_visual() -> 原 29 路 PWM / Morpheus 33 路 PWM

CAD 坐标仅写入自定义属性作几何参考，绝不用它替换骨骼 head/tail。
本脚本不自动保存；只有命令行显式加 save_mainfile 才写回 .blend。
"""

import bpy
import importlib
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm
import servo_geometry as sg
import blender_servo_bridge as bridge
importlib.reload(sm)
importlib.reload(sg)
importlib.reload(bridge)

ARM_NAME = "Feace_Rig.001"
MY_COLLECTIONS = ("Servo_Rig", "Servo_Links", "Servo_Planes")
# 只写自己新建的文本块。绝不能覆盖用户原有的 get_position / blender_receiver。
MY_TEXTS = ("morpheus_servo_bridge.py",)
STALE_TEXTS = ("get_position.py",)
BRIDGE_MARK = "原 Feace_Rig.001 骨骼 -> 原 29 路 PWM"


def remove_previous_parallel_rig():
    """只删除上一版明确创建的对象/集合，不碰用户原对象。"""
    removed_objects = []
    for collection_name in MY_COLLECTIONS:
        collection = bpy.data.collections.get(collection_name)
        if collection:
            for obj in list(collection.objects):
                removed_objects.append(obj.name)
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(collection)
    # 防御：集合已被手工移走时，也按唯一名称删除上一版独立骨架。
    obj = bpy.data.objects.get("Morpheus_Servo_Rig")
    if obj:
        removed_objects.append(obj.name)
        bpy.data.objects.remove(obj, do_unlink=True)
    return removed_objects


def remove_previous_placeholder_bones(arm):
    """删除上一版 Servo_XX 占位骨；原 46 根骨骼一律不动。"""
    names = [b.name for b in arm.data.bones if b.name.startswith("Servo_")]
    if not names:
        return []
    previous_active = bpy.context.view_layer.objects.active
    try:
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        if arm.mode != "EDIT":
            bpy.ops.object.mode_set(mode="EDIT")
        for name in names:
            edit_bone = arm.data.edit_bones.get(name)
            if edit_bone:
                arm.data.edit_bones.remove(edit_bone)
        bpy.ops.object.mode_set(mode="OBJECT")
    finally:
        if previous_active:
            bpy.context.view_layer.objects.active = previous_active
    return names


def clear_previous_mapping_props(arm):
    for bone in arm.data.bones:
        for key in list(bone.keys()):
            if key.startswith("morpheus_") or key in {
                "servo_id", "pwm_low", "pwm_start", "pwm_high", "pwm_direction"
            }:
                del bone[key]


def restore_overwritten_texts():
    """上一版曾把 get_position 覆盖成桥接脚本；用审计提取的原文恢复。"""
    restored = []
    text = bpy.data.texts.get("get_position")
    backup = ROOT / "embedded_get_position.py"
    if text is not None and BRIDGE_MARK in text.as_string() and backup.exists():
        text.clear()
        text.write(backup.read_text(encoding="utf-8"))
        restored.append("get_position")
    for name in STALE_TEXTS:
        stale = bpy.data.texts.get(name)
        if stale is not None and BRIDGE_MARK in stale.as_string():
            bpy.data.texts.remove(stale)
            restored.append("removed:" + name)
    return restored


def embed_bridge_text():
    source = (ROOT / "blender_servo_bridge.py").read_text(encoding="utf-8")
    for name in MY_TEXTS:
        text = bpy.data.texts.get(name) or bpy.data.texts.new(name)
        text.clear()
        text.write(source)


def main():
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None or arm.type != "ARMATURE":
        raise RuntimeError("找不到原骨架 " + ARM_NAME)

    original_heads = {b.name: tuple(b.head_local) for b in arm.data.bones}
    original_tails = {b.name: tuple(b.tail_local) for b in arm.data.bones}
    original_constraints = {
        pb.name: [(c.type, c.influence) for c in pb.constraints] for pb in arm.pose.bones
    }
    original_driver_count = len(arm.animation_data.drivers) if arm.animation_data else 0

    removed_objects = remove_previous_parallel_rig()
    removed_bones = remove_previous_placeholder_bones(arm)
    clear_previous_mapping_props(arm)

    missing_sources = []
    for servo in sm.SERVOS:
        geometry = sg.SERVO_MM[servo["id"]]
        for bone_name in servo["bones"]:
            bone = arm.data.bones.get(bone_name)
            if bone is None:
                missing_sources.append({"id": servo["id"], "bone": bone_name})
                continue
            # 只写元数据；servo_settings 字典和骨骼几何完全不写。
            bone["morpheus_id"] = servo["id"]
            bone["morpheus_name"] = servo["name"]
            bone["morpheus_xyz_mm"] = list(geometry["xyz_mm"])
            bone["morpheus_geometry_confidence"] = geometry["confidence"]
            bone["morpheus_geometry_role"] = "CAD metadata only; original bone motion is authoritative"
    if missing_sources:
        raise RuntimeError("原骨骼映射缺失: " + json.dumps(missing_sources, ensure_ascii=False))

    arm["morpheus_servo_map"] = sm.PAPER
    arm["morpheus_control_source"] = "original 46 bones + original servo_settings + matrix_visual"
    arm["morpheus_geometry_policy"] = "CAD mm metadata only; never move original bones"
    arm["morpheus_geometry_json"] = json.dumps(sg.geometry_table(), ensure_ascii=False)
    arm["morpheus_neutral_only_ids"] = [s["id"] for s in sm.SERVOS if not s["bones"]]

    restored_texts = restore_overwritten_texts()
    embed_bridge_text()
    handler_count = bridge.install_live_handler()
    realtime_installed = bridge.install_realtime_handler()

    # 核心回归断言：原骨骼位置、约束和驱动器必须没有变化。
    for bone in arm.data.bones:
        if bone.name in original_heads:
            assert tuple(bone.head_local) == original_heads[bone.name], bone.name + " head changed"
            assert tuple(bone.tail_local) == original_tails[bone.name], bone.name + " tail changed"
    for pb in arm.pose.bones:
        if pb.name in original_constraints:
            assert [(c.type, c.influence) for c in pb.constraints] == original_constraints[pb.name]
    current_driver_count = len(arm.animation_data.drivers) if arm.animation_data else 0
    assert current_driver_count == original_driver_count, "原骨骼驱动器数量变化"

    report = {
        "status": "ORIGINAL_RIG_RESTORED",
        "source_blend": bpy.data.filepath,
        "armature": ARM_NAME,
        "original_bone_count": len(arm.data.bones),
        "original_driver_count": current_driver_count,
        "original_geometry_unchanged": True,
        "original_constraints_unchanged": True,
        "legacy_output_channels": len(bridge.LEGACY_BONES),
        "morpheus_output_channels": len(sm.SERVOS),
        "mapped_morpheus_channels": sum(1 for s in sm.SERVOS if s["bones"]),
        "neutral_only_channels": [s["id"] for s in sm.SERVOS if not s["bones"]],
        "removed_parallel_objects": removed_objects,
        "removed_placeholder_bones": removed_bones,
        "restored_texts": restored_texts,
        "protected_texts": ["get_position", "blender_receiver.py"],
        "live_frame_handlers": handler_count,
        "realtime_shape_key_timer": realtime_installed,
        "bone_map": [
            {
                "id": s["id"], "name": s["name"], "source_bones": s["bones"],
                "lower": s["lower"], "start": s["start"], "upper": s["upper"],
                "cad_xyz_mm": list(sg.SERVO_MM[s["id"]]["xyz_mm"]),
                "geometry_confidence": sg.SERVO_MM[s["id"]]["confidence"],
            }
            for s in sm.SERVOS
        ],
        "current_morpheus_pwm": bridge.extract_morpheus_frame(),
    }
    assert report["original_bone_count"] == 46, "必须保持原 46 根骨骼"
    assert report["mapped_morpheus_channels"] == 25
    assert report["neutral_only_channels"] == [1, 2, 11, 12, 13, 18, 29, 30]

    output = ROOT / "servo_map_report.json"
    output.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print("APPLY_OK", json.dumps({k: v for k, v in report.items() if k != "bone_map"}, ensure_ascii=False))
    print("REPORT", output)
    return report


# BlenderMCP exec() 时 __name__ 不一定是 __main__，因此直接调用。
main()
