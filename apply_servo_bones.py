"""在原 Feace_Rig.001 骨架内新增 33 根真实舵机骨。

与上一版错误方案的区别：
  - 不创建第二套 Armature，所有骨骼都在 Feace_Rig.001 内。
  - 原 46 根骨骼的 head/tail/约束/驱动器一律不动。
  - 新骨骼落在 SolidWorks 装配实测的舵机几何中心。
  - 有原骨骼来源的通道，用 Copy Rotation(LOCAL/LOCAL) 承接原骨骼旋转，
    所以播放 ARKit 时舵机骨会跟着原链路一起动；没有来源的 8 路保持静止。

用法：
  blender --background LY_GENERAL_V0.blend --python apply_servo_bones.py \
          --python-expr "import bpy; bpy.ops.wm.save_mainfile()"
"""

import bpy
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm
import servo_geometry as sg

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"
GEO = ROOT / "cad_geometry"
CAD_FACE_ORIGIN_MM = 70.0 / 2.0  # 两眼球中心距 70mm


# ----------------------------------------------------------------- CAD 坐标
def world_to_face(p):
    """CAD world (mm) -> face 坐标（原点=眼中点，+X 左，+Y 后，+Z 上）。"""
    x, y, z = p
    return (z - 1.0, x + 96.84, y - 285.87)


def load_cad_centers():
    """33 路 -> CAD 舵机 bbox 中心（face mm）。"""
    instances = json.loads(
        (GEO / "assembly_instances.json").read_text(encoding="utf-8"))["instances"]
    mapping = json.loads(
        (GEO / "servo_id_mapping.json").read_text(encoding="utf-8"))["by_id"]
    by_ref = {}
    for item in instances:
        if not any(k in item["file"].lower() for k in ("mg90", "a0090", "ks3518")):
            continue
        # 零件原点就是舵机输出轴的设计基准；bbox 中心会偏到本体中部。
        center = item["world_t_mm"]
        by_ref[item["ref_id"]] = {
            "face_mm": world_to_face(center),
            "file": item["file"],
            "owner": item["owner_model"],
            "ref_id": item["ref_id"],
        }
    out = {}
    for sid in range(1, 34):
        rec = mapping.get(str(sid))
        ref = rec.get("cad_ref_id") if rec else None
        if ref and ref in by_ref:
            out[sid] = dict(by_ref[ref])
            out[sid]["confidence"] = rec.get("confidence", "cad_measured")
        else:
            # 31 号在 CAD 中没有独立实例，用估计坐标
            out[sid] = {
                "face_mm": sg.SERVO_MM[sid]["xyz_mm"],
                "file": None,
                "owner": sg.SERVO_MM[sid]["module"],
                "ref_id": None,
                "confidence": "estimated_no_cad_instance",
            }
    return out


def build_servo_table(arm):
    """CAD face mm -> Blender 坐标，使用真实眼距 + 整体 Z 比例。"""
    eye_l = arm.data.bones["Eye_yaw.l"].head_local
    eye_r = arm.data.bones["Eye_yaw.r"].head_local
    eye_mid = tuple((eye_l[i] + eye_r[i]) / 2.0 for i in range(3))
    eye_span = abs(eye_l[0] - eye_r[0])
    if eye_span <= 0:
        raise RuntimeError("左右眼骨骼 x 重合，无法用眼距定标")

    fit = sg.fit_anchors(sg.read_anchors_from_bpy())
    sx = eye_span / (CAD_FACE_ORIGIN_MM * 2.0)          # unit per mm = 眼距(unit)/70mm
    sz = fit["axis"]["Z"]["scale"]                       # 整体高度比例 unit/mm
    sy = sz                                              # 深度与高度同尺度

    def to_blender(mm):
        return (eye_mid[0] + mm[0] * sx,
                eye_mid[1] + mm[1] * sy,
                eye_mid[2] + mm[2] * sz)

    cad = load_cad_centers()
    rows = []
    for servo in sm.SERVOS:
        sid = servo["id"]
        info = cad[sid]
        rows.append({
            "id": sid,
            "name": servo["name"],
            "name_cn": servo["name_cn"],
            "module": servo["group"],
            "lower": servo["lower"], "start": servo["start"], "upper": servo["upper"],
            "sources": servo["bones"],
            "cad_face_mm": [round(v, 3) for v in info["face_mm"]],
            "cad_file": info["file"],
            "cad_ref": info["ref_id"],
            "xyz_blender": list(to_blender(info["face_mm"])),
            "confidence": info["confidence"],
        })
    return rows, {"eye_mid": eye_mid, "eye_span": eye_span,
                  "unit_per_mm_x": sx, "unit_per_mm_y": sy, "unit_per_mm_z": sz,
                  "fit": fit}


# ----------------------------------------------------------------- 骨骼
def remove_previous(arm):
    names = [b.name for b in arm.data.bones if b.name.startswith(PREFIX)]
    if not names:
        return []
    previous = bpy.context.view_layer.objects.active
    try:
        arm.select_set(True)
        bpy.context.view_layer.objects.active = arm
        if arm.mode != "EDIT":
            bpy.ops.object.mode_set(mode="EDIT")
        for name in names:
            eb = arm.data.edit_bones.get(name)
            if eb:
                arm.data.edit_bones.remove(eb)
        bpy.ops.object.mode_set(mode="OBJECT")
    finally:
        if previous:
            bpy.context.view_layer.objects.active = previous
    return names


def ensure_edit_mode(arm):
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    if arm.mode != "EDIT":
        bpy.ops.object.mode_set(mode="EDIT")


def create_bones(arm, rows):
    original = {b.name: (tuple(b.head_local), tuple(b.tail_local),
                         b.parent.name if b.parent else None) for b in arm.data.bones}
    ensure_edit_mode(arm)
    edit = arm.data.edit_bones
    created = []
    for row in rows:
        name = PREFIX + "%02d" % row["id"]
        bone = edit.new(name)
        head = row["xyz_blender"]
        bone.head = head

        source = row["sources"][0] if row["sources"] else None
        if source and source in edit:
            src = edit[source]
            direction = (src.tail - src.head)
            length = direction.length
            if length < 1e-6:
                direction = mathutils_z()
                length = 0.05
            bone.tail = (head[0] + direction.x / length * 0.12,
                         head[1] + direction.y / length * 0.12,
                         head[2] + direction.z / length * 0.12)
            bone.parent = src.parent
            # rest 朝向必须与原骨一致，Copy Rotation(LOCAL) 才等价于平移后的原骨。
            bone.roll = src.roll
        else:
            bone.tail = (head[0], head[1], head[2] + 0.12)
            bone.parent = edit.get("Skull")
            bone.roll = 0.0
        bone.use_connect = False
        bone.use_deform = False
        created.append({"name": name, "source": source})
    bpy.ops.object.mode_set(mode="OBJECT")

    # 原骨骼必须完全没变
    for name, (head, tail, parent) in original.items():
        b = arm.data.bones[name]
        assert tuple(b.head_local) == head, name + " head changed"
        assert tuple(b.tail_local) == tail, name + " tail changed"
        assert (b.parent.name if b.parent else None) == parent, name + " parent changed"
    return created


def mathutils_z():
    from mathutils import Vector
    return Vector((0.0, 0.0, 1.0))


def add_constraints(arm, rows):
    added = []
    for row in rows:
        if not row["sources"]:
            continue
        name = PREFIX + "%02d" % row["id"]
        pb = arm.pose.bones[name]
        for c in list(pb.constraints):
            pb.constraints.remove(c)
        c = pb.constraints.new("COPY_ROTATION")
        c.name = "follow_" + row["sources"][0]
        c.target = arm
        c.subtarget = row["sources"][0]
        c.target_space = "LOCAL"
        c.owner_space = "LOCAL"
        c.mix_mode = "REPLACE"
        c.influence = 1.0
        added.append({"bone": name, "source": row["sources"][0]})
    return added


def write_props(arm, rows):
    for row in rows:
        name = PREFIX + "%02d" % row["id"]
        b = arm.data.bones[name]
        b["servo_id"] = row["id"]
        b["servo_name"] = row["name"]
        b["servo_name_cn"] = row["name_cn"]
        b["servo_module"] = row["module"]
        b["pwm_lower"] = row["lower"]
        b["pwm_start"] = row["start"]
        b["pwm_upper"] = row["upper"]
        b["source_bones"] = list(row["sources"])
        b["cad_face_mm"] = list(row["cad_face_mm"])
        b["cad_file"] = row["cad_file"] or ""
        b["cad_ref"] = str(row["cad_ref"] or "")
        b["geometry_confidence"] = row["confidence"]
        b["id_semantic_confidence"] = (
            "table_v_name_plus_cad_geometry" if row["module"] != "mouth"
            else "geometric_inference_needs_physical_check")
    # 方便在视口区分的颜色（Blender 4.x+）
    try:
        for row in rows:
            arm.data.bones[PREFIX + "%02d" % row["id"]].color.palette = "THEME01"
    except AttributeError:
        pass


def main():
    arm = bpy.data.objects.get(ARM_NAME)
    if arm is None or arm.type != "ARMATURE":
        raise RuntimeError("找不到原骨架 " + ARM_NAME)

    rows, calibration = build_servo_table(arm)
    for row in rows:
        x, y, z = row["xyz_blender"]
        assert abs(x) < 2.0 and abs(y) < 2.0 and 13.0 < z < 19.0, (
            "舵机坐标越界 M%02d %s" % (row["id"], row["xyz_blender"]))
    removed = remove_previous(arm)
    created = create_bones(arm, rows)
    constraints = add_constraints(arm, rows)
    write_props(arm, rows)

    arm["morpheus_servo_bones"] = len(rows)
    arm["morpheus_servo_bone_policy"] = (
        "33 bones inside Feace_Rig.001 at CAD servo centers; "
        "original 46 bones untouched; Copy Rotation drives them from original bones")
    arm["morpheus_servo_calibration"] = json.dumps(calibration, ensure_ascii=False)

    report = {
        "status": "SERVO_BONES_CREATED",
        "armature": ARM_NAME,
        "original_bones": 46,
        "servo_bones": len(created),
        "removed_previous": removed,
        "constraints": len(constraints),
        "unmapped_channels": [r["id"] for r in rows if not r["sources"]],
        "calibration": calibration,
        "rows": rows,
    }
    out = ROOT / "servo_bones_report.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print("SERVO_BONES_OK", json.dumps({
        "bones": len(created), "constraints": len(constraints),
        "unmapped": report["unmapped_channels"],
        "unit_per_mm_x": round(calibration["unit_per_mm_x"], 6),
        "unit_per_mm_z": round(calibration["unit_per_mm_z"], 6),
    }, ensure_ascii=False))
    print("REPORT", out)


main()
