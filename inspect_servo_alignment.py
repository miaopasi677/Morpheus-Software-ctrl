"""审计：CAD 真实舵机坐标 vs 原骨骼位置，以及 Targets 的父级关系。

只读，不修改场景。
"""

import bpy
import json
import pathlib
import sys

ROOT = pathlib.Path(bpy.data.filepath).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_geometry as sg

ARM_NAME = "Feace_Rig.001"


def vec(v):
    return [round(float(x), 6) for x in v]


def main():
    arm = bpy.data.objects[ARM_NAME]
    bones = arm.data.bones

    eye_heads = {}
    for name in ("Eye_yaw.l", "Eye_yaw.r", "Eyelid_Up.l", "Eyelid_Up.r"):
        b = bones.get(name)
        eye_heads[name] = vec(b.head_local) if b else None

    left = eye_heads.get("Eye_yaw.l")
    right = eye_heads.get("Eye_yaw.r")
    span = abs(left[0] - right[0]) if left and right else None
    eye_mid = [(left[i] + right[i]) / 2.0 for i in range(3)] if left and right else None
    scale_eye = (70.0 / span) if span else None  # CAD 眼距 70mm

    fit = sg.fit_anchors(sg.read_anchors_from_bpy())

    rows = []
    for sid in range(1, 34):
        mm = tuple(sg.SERVO_MM[sid]["xyz_mm"])
        p_fit = sg.to_blender(mm, fit)
        p_eye = tuple(eye_mid[i] + mm[i] / scale_eye for i in range(3)) if eye_mid else None

        nearest = []
        for b in bones:
            d = sum((b.head_local[i] - p_eye[i]) ** 2 for i in range(3)) ** 0.5 if p_eye else None
            nearest.append((d, b.name, vec(b.head_local)))
        nearest.sort(key=lambda x: x[0])

        rows.append({
            "id": sid,
            "name": sg.SERVO_MM[sid]["note"],
            "module": sg.SERVO_MM[sid]["module"],
            "mm": list(mm),
            "blender_fit": list(p_fit),
            "blender_eye_scale": list(p_eye) if p_eye else None,
            "nearest_bone": nearest[0][1],
            "nearest_dist_unit": round(nearest[0][0], 5),
            "nearest_dist_mm_eye": round(nearest[0][0] * scale_eye, 1) if scale_eye else None,
            "nearest3": [[n, round(d, 5)] for d, n, _ in nearest[:3]],
        })

    targets = []
    for obj in bpy.data.objects:
        if not obj.name.endswith("E") and not obj.name.endswith(".L") and not obj.name.endswith(".R"):
            continue
        if obj.type != "EMPTY":
            continue
        inv = obj.matrix_parent_inverse
        targets.append({
            "name": obj.name,
            "parent": obj.parent.name if obj.parent else None,
            "parent_type": obj.parent_type,
            "parent_bone": obj.parent_bone,
            "parent_inverse_identity": all(
                abs(inv[r][c] - (1.0 if r == c else 0.0)) < 1e-6
                for r in range(4) for c in range(4)),
            "location": vec(obj.location),
            "constraints": [
                {"type": c.type, "target": getattr(c, "target", None),
                 "subtarget": getattr(c, "subtarget", None),
                 "influence": c.influence}
                for c in obj.constraints
            ],
            "drivers": [],
        })
        if obj.animation_data:
            for fc in obj.animation_data.drivers:
                targets[-1]["drivers"].append(fc.data_path)

    bone_rows = []
    driver_paths = (
        [fc.data_path for fc in arm.animation_data.drivers]
        if arm.animation_data else []
    )
    for b in bones:
        pb = arm.pose.bones[b.name]
        bone_rows.append({
            "name": b.name,
            "parent": b.parent.name if b.parent else None,
            "head_local": vec(b.head_local),
            "tail_local": vec(b.tail_local),
            "use_deform": b.use_deform,
            "use_connect": b.use_connect,
            "constraints": [c.type for c in pb.constraints],
            "driver_count": sum(1 for p in driver_paths if ('"%s"' % b.name) in p),
            "has_servo_id": "servo_settings" in b.keys() and "servo_id" in b["servo_settings"],
            "servo_id": b["servo_settings"].get("servo_id") if "servo_settings" in b.keys() else None,
            "vertex_groups": [],
        })

    # 哪个 mesh 用了哪些骨骼顶点组
    vg_usage = {}
    for obj in bpy.data.objects:
        if obj.type != "MESH":
            continue
        for vg in obj.vertex_groups:
            name = vg.name
            if name not in vg_usage:
                vg_usage[name] = []
            vg_usage[name].append(obj.name)
    for row in bone_rows:
        row["vertex_groups"] = sorted(vg_usage.get(row["name"], []))

    payload = {
        "blend": bpy.data.filepath,
        "bone_count": len(bones),
        "eye_heads": eye_heads,
        "eye_span_unit": span,
        "eye_span_mm": 70.0,
        "mm_per_unit_by_eye": scale_eye,
        "fit": fit,
        "servo_rows": rows,
        "targets": targets,
        "bones": bone_rows,
    }
    out = ROOT / "servo_alignment_audit.json"
    out.write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding="utf-8")
    print("AUDIT_OK", out)
    print("EYE_SPAN", span, "MM_PER_UNIT", scale_eye)
    print("FIT", json.dumps(fit["mm_per_blender_unit"], ensure_ascii=False))
    for row in rows:
        print("M%02d %-14s nearest=%-22s d=%.4f unit (%.1f mm) fit=%s eye=%s" % (
            row["id"], row["module"], row["nearest_bone"], row["nearest_dist_unit"],
            row["nearest_dist_mm_eye"], [round(v, 3) for v in row["blender_fit"]],
            [round(v, 3) for v in row["blender_eye_scale"]]))


main()
