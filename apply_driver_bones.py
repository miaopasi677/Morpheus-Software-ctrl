"""把 33 路骨骼放到「表皮驱动控制点」上，并保留原骨架的 ARKit 控制链。

设计要点
--------
舵机是动力源，骨骼位置应取**传动末端作用在皮肤上的点**：
    舵机 -> 曲柄 -> 连杆 / 球铰 / 钢丝 / 齿条 -> 表皮附着点

位置的优先级：
  1. Targets 控制点对象（cheekE.L / lipUpE.C ...），它们本身是 vertex-parent
     到 SampleHead 的皮肤顶点，就是原模型里已验证的控制点；
  2. 原骨骼 head（控制点缺失时，如舌 / 下巴 / 颈）；
  3. CAD 传动链末端（servo_drive_chains.json）：仅当它偏离上述控制点
     <= MAX_CAD_OFFSET_MM 时才用于细化，避免把追踪歧义写进骨骼。

骨骼朝向 = 由表皮点指向舵机轴心，表示该路的传动方向。
原 46 根骨骼的 head/tail/约束/驱动器一律不动。

运行：
  blender --background LY_GENERAL_V0.blend --python apply_driver_bones.py \
          --python-expr "import bpy; bpy.ops.wm.save_mainfile()"
"""

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
import servo_geometry as sg

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"
GEO = ROOT / "cad_geometry"
CAD_FACE_ORIGIN_MM = 35.0
MAX_CAD_OFFSET_MM = 45.0
BONE_LEN = 0.16

# 没有 Targets 控制点的通道，指定语义上最接近的皮肤控制点兜底
FALLBACK_CONTROL = {
    11: "lipDownE.C",   # 舌伸缩：下唇中央后方
    12: "lipDownE.C",   # 舌上下
    13: "jawE",         # 下巴 被动
    18: "jawE",         # 下巴 主动
}

# 右侧通道统一由左侧控制点镜像，保证左右对称，
# 同时修掉原模型里放错位的 noseE.R（落在眉心）与偏外的 eyeE.R。
MIRROR_FROM_LEFT = {
    10: "lipUpE.L", 17: "lipDownE.L", 19: "lipConnerE.L", 22: "lipConnerE.L",
    25: "eyeLidDown.L", 26: "eyeLidUpE.L",
    27: "browOuterE.L", 28: "browInnerE.L", 29: "noseE.L", 30: "cheekE.L",
}

# 眼球联动两路必须共用同一控制点，不能各自被 CAD 点带偏
NO_CAD = {7, 24}

# 同一控制点被多路共用时，按语义在局部错开（Blender unit，1 unit ≈ 94mm）
CHANNEL_OFFSET = {
    9: (0.0, 0.0, 0.030), 14: (0.0, 0.0, -0.030),     # 左嘴角 上/下
    22: (0.0, 0.0, 0.030), 19: (0.0, 0.0, -0.030),    # 右嘴角 上/下
    11: (0.0, 0.055, 0.0), 12: (0.0, 0.085, -0.020),  # 舌 伸缩/上下
    13: (0.0, 0.0, -0.100), 18: (0.0, 0.0, -0.150),   # 下巴 被动/主动
    20: (0.0, 0.0, -0.040), 23: (0.0, 0.0, -0.070),   # 下颌 左右/张合
    24: (0.0, 0.020, -0.030),                          # 眼球水平：与俯仰同一眼球
}

# 7/24 是同一只眼球的两个自由度（联动舵机），统一取左眼，避免落到两眼之间
FORCE_CONTROL = {24: "eyeE.L"}

# 右侧通道 -> 左侧通道；右侧位置严格由左侧结果镜像，保证完全对称
MIRROR_PAIR = {10: 21, 17: 16, 19: 14, 22: 9, 24: 7, 25: 6, 26: 5,
               27: 4, 28: 3, 29: 2, 30: 1}

NECK_IDS = (31, 32, 33)
NECK_OFFSET = {31: 0.0, 32: 0.09, 33: 0.16}

# 颈部是关节不是皮肤，不能投影到脸皮
NO_PROJECTION = set(NECK_IDS)


# ------------------------------------------------------------------ 坐标
def build_transform(arm):
    eye_l = arm.data.bones["Eye_yaw.l"].head_local
    eye_r = arm.data.bones["Eye_yaw.r"].head_local
    eye_mid = Vector(((eye_l[0] + eye_r[0]) / 2.0,
                      (eye_l[1] + eye_r[1]) / 2.0,
                      (eye_l[2] + eye_r[2]) / 2.0))
    eye_span = abs(eye_l[0] - eye_r[0])
    fit = sg.fit_anchors(sg.read_anchors_from_bpy())
    sx = eye_span / (CAD_FACE_ORIGIN_MM * 2.0)
    sz = fit["axis"]["Z"]["scale"]
    sy = sz

    def to_blender(mm):
        """face 坐标(mm) -> Blender 坐标。"""
        return eye_mid + Vector((mm[0] * sx, mm[1] * sy, mm[2] * sz))

    def world_to_face(p):
        """装配世界坐标(mm) -> face 坐标(mm)，原点在双眼中点。"""
        return (p[2] - 1.0, p[0] + 96.84, p[1] - 285.87)

    def cad_to_blender(p):
        return to_blender(world_to_face(p))

    cad_to_blender.face = to_blender

    return cad_to_blender, {"eye_mid": [round(v, 5) for v in eye_mid],
                        "sx": sx, "sy": sy, "sz": sz,
                        "mm_per_unit": 1.0 / sx}


def load_chains():
    path = GEO / "servo_drive_chains.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))["servos"]


def surface_objects():
    out = []
    for name in ("SampleHead", "eyes"):
        ob = bpy.data.objects.get(name)
        if ob is not None and ob.type == "MESH" and len(ob.data.vertices):
            out.append(ob)
    return out


def project_to_skin(point, objects):
    best = None
    for ob in objects:
        mw = ob.matrix_world
        local = mw.inverted() @ point
        ok, loc, nor, _idx = ob.closest_point_on_mesh(local)
        if not ok:
            continue
        world = mw @ loc
        d = (world - point).length
        if best is None or d < best[0]:
            best = (d, world, ob.name)
    return best


# ------------------------------------------------------------------ 骨骼
def remove_previous(arm):
    names = [b.name for b in arm.data.bones if b.name.startswith(PREFIX)]
    if not names:
        return []
    prev = bpy.context.view_layer.objects.active
    arm.select_set(True)
    bpy.context.view_layer.objects.active = arm
    if arm.mode != "EDIT":
        bpy.ops.object.mode_set(mode="EDIT")
    for n in names:
        eb = arm.data.edit_bones.get(n)
        if eb:
            arm.data.edit_bones.remove(eb)
    bpy.ops.object.mode_set(mode="OBJECT")
    if prev:
        bpy.context.view_layer.objects.active = prev
    return names


def target_world(name):
    if not name:
        return None
    ob = bpy.data.objects.get(name)
    if ob is None:
        return None
    return ob.matrix_world.translation.copy()


def bone_head(arm, name):
    if not name:
        return None
    b = arm.data.bones.get(name)
    return Vector(b.head_local) if b else None


def main():
    bpy.context.view_layer.update()
    arm = bpy.data.objects[ARM_NAME]
    cad_to_blender, calib = build_transform(arm)
    chains = load_chains()
    objs = surface_objects()

    original = {b.name: (tuple(b.head_local), tuple(b.tail_local),
                         b.parent.name if b.parent else None)
                for b in arm.data.bones if not b.name.startswith(PREFIX)}
    removed = remove_previous(arm)

    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    if arm.mode != "EDIT":
        bpy.ops.object.mode_set(mode="EDIT")
    edit = arm.data.edit_bones

    rows = []
    done = {}
    ordered = ([x for x in sm.SERVOS if x["id"] not in MIRROR_PAIR]
               + [x for x in sm.SERVOS if x["id"] in MIRROR_PAIR])

    for servo in ordered:
        sid = servo["id"]
        name = PREFIX + "%02d" % sid
        chain = chains.get(str(sid)) or {}
        cad_point = chain.get("driver_point_mm")
        servo_mm = chain.get("servo_t_mm")
        cp_name = (FORCE_CONTROL.get(sid) or servo.get("target")
                   or FALLBACK_CONTROL.get(sid))
        ref_bone = next((b for b in servo["bones"] if b in arm.data.bones), None)

        mirrored = sid in MIRROR_PAIR and MIRROR_PAIR[sid] in done
        if mirrored:
            src = done[MIRROR_PAIR[sid]]
            surf = Vector((-src.x, src.y, src.z))
            surf_obj = "mirror"
            surf_gap_mm = 0.0
            source = "mirror_of_M%02d" % MIRROR_PAIR[sid]
            offset_mm = None
            cad_used = False
            cp_name = "mirror:M%02d" % MIRROR_PAIR[sid]
        else:
            base = target_world(cp_name)
            source = "targets_control_point"
            if sid in NECK_IDS:
                neck = arm.data.bones.get("Face_Neck")
                if neck is not None:
                    base = Vector(neck.head_local) + Vector((0.0, 0.0, NECK_OFFSET[sid]))
                    source = "neck_joint"
            if base is not None and sid in CHANNEL_OFFSET:
                base = base + Vector(CHANNEL_OFFSET[sid])
            if base is None:
                base = bone_head(arm, ref_bone)
                source = "bone_head"
            if base is None and servo_mm:
                base = cad_to_blender(servo_mm)
                source = "servo_axis"
            if base is None:
                base = Vector((0.0, -0.35, 16.8))
                source = "fallback"

            chosen = base
            offset_mm = None
            cad_used = False
            allow_cad = sid not in NECK_IDS and sid not in NO_CAD
            if allow_cad and cad_point and str(chain.get("confidence", "")).startswith("chain_end"):
                cand = cad_to_blender(cad_point)
                offset_mm = (cand - base).length * calib["mm_per_unit"]
                if offset_mm <= MAX_CAD_OFFSET_MM:
                    chosen = cand
                    source = "cad_drive_chain"
                    cad_used = True

            proj = None if sid in NO_PROJECTION else project_to_skin(chosen, objs)
            if proj is not None:
                dist, surf, surf_obj = proj
                surf_gap_mm = dist * calib["mm_per_unit"]
            else:
                surf, surf_obj = chosen, None
                surf_gap_mm = 0.0 if sid in NO_PROJECTION else None

        done[sid] = surf.copy() if hasattr(surf, "copy") else Vector(surf)

        if servo_mm:
            drive_dir = cad_to_blender(servo_mm) - surf
        elif ref_bone:
            b = arm.data.bones[ref_bone]
            drive_dir = Vector(b.tail_local) - Vector(b.head_local)
        else:
            drive_dir = Vector((0.0, 1.0, 0.0))
        if drive_dir.length < 1e-6:
            drive_dir = Vector((0.0, 1.0, 0.0))
        drive_dir.normalize()

        bone = edit.new(name)
        bone.head = surf
        bone.tail = surf + drive_dir * BONE_LEN
        bone.use_connect = False
        bone.use_deform = False
        bone.parent = edit.get("Skull")
        bone.roll = 0.0

        rows.append({
            "id": sid, "bone": name, "name": servo["name"], "name_cn": servo["name_cn"],
            "module": servo["group"], "control_point": cp_name, "source_bone": ref_bone,
            "driver_source": source, "cad_used": cad_used,
            "cad_offset_mm": None if offset_mm is None else round(offset_mm, 1),
            "skin_object": surf_obj,
            "skin_gap_mm": None if surf_gap_mm is None else round(surf_gap_mm, 2),
            "driver_part": chain.get("driver_part"),
            "chain": [c.get("file") for c in (chain.get("chain") or [])],
            "cad_confidence": chain.get("confidence"),
            "head": [round(v, 5) for v in surf],
            "tail": [round(v, 5) for v in bone.tail],
            "servo_mm": servo_mm,
        })
    bpy.ops.object.mode_set(mode="OBJECT")

    for n, (h, t, p) in original.items():
        b = arm.data.bones[n]
        assert tuple(b.head_local) == h, n + " head changed"
        assert tuple(b.tail_local) == t, n + " tail changed"
        assert (b.parent.name if b.parent else None) == p, n + " parent changed"

    added = 0
    for row in rows:
        pb = arm.pose.bones[row["bone"]]
        for c in list(pb.constraints):
            pb.constraints.remove(c)
        if not row["source_bone"]:
            continue
        con = pb.constraints.new("COPY_ROTATION")
        con.name = "ARKit_Follow"
        con.target = arm
        con.subtarget = row["source_bone"]
        con.target_space = "LOCAL"
        con.owner_space = "LOCAL"
        con.mix_mode = "REPLACE"
        con.influence = 1.0
        added += 1

    for row in rows:
        b = arm.data.bones[row["bone"]]
        b["morpheus_id"] = row["id"]
        b["servo_name"] = row["name"]
        b["servo_name_cn"] = row["name_cn"]
        b["module"] = row["module"]
        b["control_point"] = row["control_point"] or ""
        b["source_bone"] = row["source_bone"] or ""
        b["driver_source"] = row["driver_source"]
        b["driver_part"] = row["driver_part"] or ""
        b["cad_chain_len"] = len(row["chain"])
        b["cad_confidence"] = row["cad_confidence"] or ""
        b["skin_object"] = row["skin_object"] or ""
        b["skin_gap_mm"] = row["skin_gap_mm"] if row["skin_gap_mm"] is not None else -1.0
        b["driver_head"] = row["head"]
        b["servo_axis_mm"] = row["servo_mm"] or []

    report = {"armature": ARM_NAME, "removed": removed, "calibration": calib,
              "constraints": added, "rows": rows}
    (ROOT / "driver_bones_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

    print("DRIVER_BONES_OK total=%d cad=%d cp=%d constraints=%d" % (
        len(rows),
        sum(1 for r in rows if r["driver_source"] == "cad_drive_chain"),
        sum(1 for r in rows if r["driver_source"] == "targets_control_point"),
        added))
    for r in rows:
        print("M%02d %-10s %-22s off=%6s gap=%6s part=%s" % (
            r["id"], r["name_cn"][:10], r["driver_source"],
            r["cad_offset_mm"], r["skin_gap_mm"],
            (r["driver_part"] or "-")[:40]))


if __name__ == "__main__":
    main()
