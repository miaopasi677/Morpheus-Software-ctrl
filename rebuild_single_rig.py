"""删除 46 根原骨骼，只保留 33 根 Morpheus 驱动骨，并重建全部映射。

被删除的链路：
    ARKit -> 形态键 -> Targets EMPTY -> 一级骨(COPY_LOCATION/DAMPED_TRACK)
          -> 驱动器放大 -> 二级骨旋转 -> get_position -> 29 路 PWM

重建后的链路：
    ARKit -> 形态键 -> SampleHead/eyes 形变（原机制保留）
          -> ARKIT_TO_SERVO 52x33 权重 -> 33 根骨旋转
          -> servo_map.frame_to_pwm -> 33 路硬件 PWM
    （见 morpheus_arkit_bridge.py，frame_change_post + timer 双通道刷新）

同时重建蒙皮：
  * mold 顶点组从 22 个原骨按语义重映射到 33 根新骨；
  * eyes 有完整 ARKit 形态键（含 eyeLook*），移除已失效的 Armature 修改器。
  * SampleHead 的形变来自形态键 + Facemotion_TESTAVATAR，不受影响，不改动。

运行：
  blender --background LY_GENERAL_V0.blend --python rebuild_single_rig.py \
          --python-expr "import bpy; bpy.ops.wm.save_mainfile()"
"""

import json
import pathlib
import sys

import bpy

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"

# mold 顶点组 -> 33 路语义通道
SKIN_REMAP = {
    "Brow_Out.r": 27, "Brow_In.r": 28, "Brow_Out.l": 4, "Brow_In.l": 3,
    "Eye_yaw.r": 24, "Eye_yaw.l": 7, "Eye_pitch.r": 7, "Eye_pitch.l": 7,
    "Eyelid_Up.r": 26, "Eyelid_Dwn.r": 25, "Eyelid_Up.l": 5, "Eyelid_Dwn.l": 6,
    "Lip_Up.r": 10, "Lip_Up_C": 8, "Lip_Up.l": 21,
    "Lip_Dwn.r": 17, "Lip_Dwn_C": 15, "Lip_Dwn.l": 16,
    "Lip_Corner.r": 22, "Lip_Corner.l": 9,
    "Jaw_Open": 23, "Jaw_Yaw": 20,
}
STRUCT_FALLBACK = 31       # Face_Root / Face_Neck / Skull / Neck_1 / 身体骨 -> 颈旋转


def clean_armature(arm):
    """删掉所有指向原骨的驱动器和约束。"""
    ad = arm.animation_data
    ndrv = 0
    if ad is not None:
        for fc in list(ad.drivers):
            ad.drivers.remove(fc)
            ndrv += 1
    ncon = 0
    for pb in arm.pose.bones:
        for c in list(pb.constraints):
            pb.constraints.remove(c)
            ncon += 1
    return ndrv, ncon


def delete_original_bones(arm):
    names = [b.name for b in arm.data.bones if not b.name.startswith(PREFIX)]
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    if arm.mode != "EDIT":
        bpy.ops.object.mode_set(mode="EDIT")
    for n in names:
        eb = arm.data.edit_bones.get(n)
        if eb:
            arm.data.edit_bones.remove(eb)
    bpy.ops.object.mode_set(mode="OBJECT")
    return names


def remap_skin():
    """mold 顶点组重映射 + eyes 移除失效骨架修改器。"""
    mold = bpy.data.objects.get("mold")
    eyes = bpy.data.objects.get("eyes")
    out = {"mold_groups_before": 0, "mold_groups_after": 0,
           "mold_reassigned_vertices": 0, "eyes_modifiers_removed": 0}

    if mold is not None:
        assign = {}
        old_groups = [g.name for g in mold.vertex_groups]
        out["mold_groups_before"] = len(old_groups)
        for v in mold.data.vertices:
            per = {}
            for g in v.groups:
                name = mold.vertex_groups[g.group].name
                sid = SKIN_REMAP.get(name, STRUCT_FALLBACK)
                per[sid] = per.get(sid, 0.0) + g.weight
            for sid, w in per.items():
                if w > 1e-5:
                    assign.setdefault(sid, []).append((v.index, min(1.0, w)))
        for g in list(mold.vertex_groups):
            mold.vertex_groups.remove(g)
        for sid, rows in sorted(assign.items()):
            vg = mold.vertex_groups.new(name=PREFIX + "%02d" % sid)
            for idx, w in rows:
                vg.add([idx], w, "REPLACE")
        out["mold_groups_after"] = len(mold.vertex_groups)
        out["mold_reassigned_vertices"] = sum(len(r) for r in assign.values())

    if eyes is not None:
        for m in list(eyes.modifiers):
            if m.type == "ARMATURE":
                eyes.modifiers.remove(m)
                out["eyes_modifiers_removed"] += 1
    return out


def update_texts():
    src = (ROOT / "morpheus_arkit_bridge.py").read_text(encoding="utf-8")
    t = bpy.data.texts.get("morpheus_arkit_bridge.py") or bpy.data.texts.new(
        "morpheus_arkit_bridge.py")
    t.clear()
    t.write(src)
    t.use_module = False          # 避免打开文件弹「禁止自动执行脚本」
    old = bpy.data.texts.get("morpheus_servo_bridge.py")
    if old is not None:
        bpy.data.texts.remove(old)
    return len(src)


def main():
    arm = bpy.data.objects[ARM_NAME]
    before = len(arm.data.bones)
    ndrv, ncon = clean_armature(arm)
    removed = delete_original_bones(arm)
    skin = remap_skin()
    nchars = update_texts()

    # 骨骼属性：清除指向已删骨的字段，标注重建来源
    for b in arm.data.bones:
        # 之前 33 根骨只是可视化（另有 46 根原骨负责蒙皮），现在它们是唯一骨架
        b.use_deform = True
        b["source_bone"] = ""
        b["mapping"] = "ARKIT_TO_SERVO (servo_map.py)"
        b["pwm_source"] = "servo_map.frame_to_pwm"
        b["skin_group"] = b.name
        for key in ("driver_source", "cad_offset_mm", "cad_chain_len",
                    "cad_confidence", "driver_part", "skin_object",
                    "skin_gap_mm", "control_point", "servo_axis_mm",
                    "driver_head"):
            if key in b:
                del b[key]

    # 初始化运行态
    import morpheus_arkit_bridge as bridge
    pwm = bridge.apply_frame()

    report = {
        "armature": ARM_NAME,
        "bones_before": before,
        "bones_after": len(arm.data.bones),
        "removed_bones": removed,
        "drivers_removed": ndrv,
        "constraints_removed": ncon,
        "skin": skin,
        "bridge_text_chars": nchars,
        "neutral_pwm": pwm,
        "mapping_matrix_entries": sum(len(v) for v in sm.ARKIT_TO_SERVO.values()),
    }
    (ROOT / "single_rig_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

    assert len(arm.data.bones) == 33, len(arm.data.bones)
    assert all(b.name.startswith(PREFIX) for b in arm.data.bones)
    print("SINGLE_RIG_OK bones=%d removed=%d drivers=%d constraints=%d pwm=%d" % (
        len(arm.data.bones), len(removed), ndrv, ncon, len(pwm)))
    print("SKIN", json.dumps(skin, ensure_ascii=False))
    print("NEUTRAL_PWM", pwm)


if __name__ == "__main__":
    main()
