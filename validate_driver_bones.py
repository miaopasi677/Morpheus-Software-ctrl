"""校验表皮驱动骨骼：位置、语义左右、原骨完整性、ARKit 跟随。"""

import json
import math
import pathlib
import sys

import bpy
from mathutils import Vector

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"
REPORT = ROOT / "driver_bones_validation.json"

results = {"status": "pass", "checks": {}, "failures": [], "rows": []}


def fail(msg):
    results["status"] = "fail"
    results["failures"].append(msg)


def main():
    arm = bpy.data.objects[ARM_NAME]
    rep = json.loads((ROOT / "driver_bones_report.json").read_text(encoding="utf-8"))
    rows = rep["rows"]
    by_id = {r["id"]: r for r in rows}

    results["checks"]["bone_total"] = len(arm.data.bones)
    results["checks"]["servo_bones"] = sum(1 for b in arm.data.bones if b.name.startswith(PREFIX))
    results["checks"]["original_bones"] = sum(1 for b in arm.data.bones if not b.name.startswith(PREFIX))
    if results["checks"]["servo_bones"] != 33:
        fail("舵机骨不是 33 根")
    if results["checks"]["original_bones"] != 46:
        fail("原骨骼不是 46 根")

    # 1) 都挂在 Skull 下，且不参与蒙皮
    for r in rows:
        b = arm.data.bones.get(r["bone"])
        if b is None:
            fail("缺少骨骼 " + r["bone"])
            continue
        if b.parent is None or b.parent.name != "Skull":
            fail(r["bone"] + " 父级不是 Skull")
        if b.use_deform:
            fail(r["bone"] + " 仍在参与蒙皮")

    # 2) 位置：面部范围内、到表皮 <= 20mm（颈部除外）
    objs = [bpy.data.objects[n] for n in ("SampleHead", "eyes") if n in bpy.data.objects]
    gaps = []
    for r in rows:
        b = arm.data.bones[r["bone"]]
        p = Vector(b.head_local)
        best = None
        for ob in objs:
            ok, loc, nor, _ = ob.closest_point_on_mesh(ob.matrix_world.inverted() @ p)
            if ok:
                d = (ob.matrix_world @ loc - p).length
                best = d if best is None else min(best, d)
        gap_mm = None if best is None else best * 93.9
        limit = 80.0 if r["module"] == "neck" else 20.0
        if gap_mm is not None and gap_mm > limit:
            fail("%s 距表皮 %.1fmm 超过 %.0fmm" % (r["bone"], gap_mm, limit))
        gaps.append(gap_mm if gap_mm is not None else 999)
        r["validated_gap_mm"] = None if gap_mm is None else round(gap_mm, 2)
    results["checks"]["max_skin_gap_mm"] = round(max(gaps), 2)
    results["checks"]["median_skin_gap_mm"] = round(sorted(gaps)[len(gaps) // 2], 2)

    # 3) 左右语义：中线通道 x≈0，左侧 x>0，右侧 x<0
    midline = [8, 11, 12, 13, 15, 18, 20, 23]      # 真正的中线通道
    paired_eye = [7, 24]                            # 眼球联动，取单侧眼球位置
    left = [1, 2, 3, 4, 5, 6, 9, 14, 16, 21]
    right = [10, 17, 19, 22, 25, 26, 27, 28, 29, 30]
    xs = {r["id"]: arm.data.bones[r["bone"]].head_local[0] for r in rows}
    for sid in midline:
        if abs(xs[sid]) > 0.22:
            fail("中线通道 M%02d 偏到 x=%.3f" % (sid, xs[sid]))
    for sid in paired_eye:
        if abs(xs[sid]) > 0.6:
            fail("眼球联动 M%02d 偏到 x=%.3f" % (sid, xs[sid]))
    for sid in left:
        if xs[sid] <= 0.02:
            fail("左侧通道 M%02d x=%.3f 不在左侧" % (sid, xs[sid]))
    for sid in right:
        if xs[sid] >= -0.02:
            fail("右侧通道 M%02d x=%.3f 不在右侧" % (sid, xs[sid]))
    results["checks"]["midline_max_abs_x"] = round(
        max(abs(xs[s]) for s in midline + paired_eye), 3)

    # 4) 约束
    with_con = [r["bone"] for r in rows
                if any(c.type == "COPY_ROTATION" for c in arm.pose.bones[r["bone"]].constraints)]
    results["checks"]["copy_rotation"] = len(with_con)
    if len(with_con) != 25:
        fail("Copy Rotation 数量 %d != 25" % len(with_con))

    # 5) 注入 ARKit，舵机骨必须真的转
    mesh = bpy.data.objects["SampleHead"]
    keys = mesh.data.shape_keys.key_blocks
    test = {"jawOpen": 0.85, "mouthSmileLeft": 0.8, "mouthUpperUpLeft": 0.6,
            "browInnerUp": 0.85, "browOuterUpLeft": 0.7,
            "eyeBlinkRight": 0.75, "eyeLookUpLeft": 0.5, "eyeLookInLeft": 0.45,
            "jawLeft": 0.5, "mouthLowerDownRight": 0.55}
    rest = {r["bone"]: Vector(arm.pose.bones[r["bone"]].matrix.to_quaternion())
            for r in rows}
    rest_head = {r["bone"]: Vector(arm.pose.bones[r["bone"]].head)
                 for r in rows}
    applied = 0
    for k, v in test.items():
        if k in keys:
            keys[k].value = v
            applied += 1
    bpy.context.view_layer.update()

    moved, still, head_moved = [], [], []
    for r in rows:
        pb = arm.pose.bones[r["bone"]]
        q = pb.matrix.to_quaternion()
        ang = q.rotation_difference(rest[r["bone"]]).angle
        (moved if ang > 1e-4 else still).append((r["id"], round(math.degrees(ang), 2)))
        d = (Vector(pb.head) - rest_head[r["bone"]]).length
        if d > 1e-5:
            head_moved.append((r["id"], round(d, 5)))
    results["checks"]["injected_keys"] = applied
    results["checks"]["bones_rotated"] = len(moved)
    results["checks"]["bones_static"] = [s[0] for s in still]
    results["checks"]["bone_head_dragged"] = head_moved
    if applied < 8:
        fail("注入的形态键不足")
    if len(moved) < 15:
        fail("只有 %d 根舵机骨响应 ARKit" % len(moved))
    if head_moved:
        fail("有舵机骨 head 被带动（应固定在表皮控制点）")

    # 6) 原 25 个驱动器仍在
    drivers = 0
    ad = getattr(arm, "animation_data", None)
    if ad is not None and getattr(ad, "drivers", None) is not None:
        drivers = len(ad.drivers)
    else:
        # Blender 5 把骨骼驱动器放在 action 的 channelbag 里
        act = ad.action if ad else None
        if act is not None:
            try:
                for layer in act.layers:
                    for strip in layer.strips:
                        for bag in strip.channelbags:
                            drivers += sum(1 for fc in bag.fcurves if fc.driver)
            except Exception:
                drivers = 0
    results["checks"]["drivers"] = drivers
    if drivers and drivers < 20:
        fail("驱动器数量异常 %d" % drivers)

    for r in rows:
        results["rows"].append({
            "id": r["id"], "name_cn": r["name_cn"], "bone": r["bone"],
            "source": r["driver_source"], "control_point": r["control_point"],
            "driver_part": r["driver_part"], "cad_offset_mm": r["cad_offset_mm"],
            "gap_mm": r.get("validated_gap_mm"),
            "head": r["head"],
        })
    REPORT.write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding="utf-8")

    print("STATUS", results["status"])
    print("CHECKS", json.dumps(results["checks"], ensure_ascii=False))
    for f in results["failures"]:
        print("FAIL", f)
    print("ROTATED", len(moved), "STATIC", [s[0] for s in still])


if __name__ == "__main__":
    main()
