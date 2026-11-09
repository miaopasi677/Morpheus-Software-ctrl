"""审计当前 LY_GENERAL_V0.blend 的 33 根映射骨：位置 + 真实运动轨迹。

只读打开文件，不保存。输出 mapping_audit.json。

检查三件事：
  1. 骨骼 head 到底落在哪里：距 CAD 舵机轴心 / 距 CAD 传动末端各多少 mm；
     tail 是否指向舵机（视觉上的「舵机方向」）。
  2. 注入 ARKit 后骨骼怎么动：head 是否固定、绕什么轴转、转角多大。
  3. 蒙皮顶点（mold）的实际位移方向 vs 机构真实拉动方向
     （线驱/连杆末端应沿传动件方向被拉向舵机侧；下颌应绕铰链向下后）。

运行：
  blender --background LY_GENERAL_V0.blend --python audit_mapping.py
"""

import bpy
import json
import math
import pathlib
import sys
from mathutils import Vector

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"

arm = bpy.data.objects[ARM_NAME]
chains = json.loads((ROOT / "cad_geometry" / "servo_drive_chains.json").read_text(
    encoding="utf-8"))["servos"]
drv = json.loads((ROOT / "driver_bones_report.json").read_text(encoding="utf-8"))
calib = drv["calibration"]
EYE_MID = Vector(calib["eye_mid"])
SX, SY, SZ = calib["sx"], calib["sy"], calib["sz"]
MM_PER_UNIT = calib["mm_per_unit"]
DRV_HEADS = {r["id"]: Vector(r["head"]) for r in drv["rows"]}


def b2face(p):
    """Blender unit -> face mm（原点=眼中点, +X 人物左, +Y 脑后, +Z 上）。"""
    return Vector(((p[0] - EYE_MID[0]) / SX,
                   (p[1] - EYE_MID[1]) / SY,
                   (p[2] - EYE_MID[2]) / SZ))


def face_dir_to_blender(d):
    return Vector((d[0] * SX, d[1] * SY, d[2] * SZ)).normalized()


def cadworld_to_blender(w):
    f = Vector((w[2] - 1.0, w[0] + 96.84, w[1] - 285.87))
    return EYE_MID + Vector((f[0] * SX, f[1] * SY, f[2] * SZ))


def ang_deg(a, b):
    if a.length < 1e-9 or b.length < 1e-9:
        return None
    return round(math.degrees(a.angle(b)), 2)


# --------------------------------------------------------------- 静态位置
static_rows = []
for b in sorted(arm.data.bones, key=lambda x: x.name):
    sid = int(b.name.rsplit("_", 1)[1])
    ch = chains.get(str(sid)) or {}
    head, tail = Vector(b.head_local), Vector(b.tail_local)
    servo, driver = ch.get("servo_t_mm"), ch.get("driver_point_mm")
    row = {
        "id": sid,
        "head": [round(v, 5) for v in head],
        "head_face_mm": [round(v, 2) for v in b2face(head)],
        "parent": b.parent.name if b.parent else None,
        "use_deform": bool(b.use_deform),
        "len_mm": round((tail - head).length * MM_PER_UNIT, 2),
        "head_matches_driver_report": (
            round((head - DRV_HEADS[sid]).length, 7) if sid in DRV_HEADS else None),
        "dist_head_to_servo_axis_mm": (
            round((head - cadworld_to_blender(servo)).length * MM_PER_UNIT, 2)
            if servo else None),
        "dist_head_to_cad_driver_mm": (
            round((head - cadworld_to_blender(driver)).length * MM_PER_UNIT, 2)
            if driver else None),
        "tail_to_servo_angle_deg": (
            ang_deg(tail - head, cadworld_to_blender(servo) - head)
            if servo else None),
        "cad_driver_part": ch.get("driver_part"),
        "cad_confidence": ch.get("confidence"),
        "props": {k: (list(v) if isinstance(v, (list, tuple)) else
                      v if not hasattr(v, "to_list") else v.to_list())
                  for k, v in b.items()},
    }
    static_rows.append(row)

# --------------------------------------------------------------- 场景检查
mold = bpy.data.objects.get("mold")
eyes = bpy.data.objects.get("eyes")
scene_info = {
    "armature_display": arm.display_type,
    "bone_count": len(arm.data.bones),
    "pose_constraints": sum(len(pb.constraints) for pb in arm.pose.bones),
    "drivers": len(arm.animation_data.drivers) if arm.animation_data else 0,
    "mold_modifiers": [(m.name, m.type,
                        getattr(m, "object", None).name if getattr(m, "object", None) else None)
                       for m in mold.modifiers] if mold else [],
    "eyes_modifiers": [(m.name, m.type) for m in eyes.modifiers] if eyes else [],
    "custom_shapes": [pb.name for pb in arm.pose.bones if pb.custom_shape],
    "texts": [t.name for t in bpy.data.texts],
}

# --------------------------------------------------------------- 运动测试
import morpheus_arkit_bridge as bridge
bridge.emit_pwm = lambda pwm: None          # 审计期间不发 UDP

import servo_map as sm
ks = bpy.data.objects["SampleHead"].data.shape_keys.key_blocks


def set_arkit(vals):
    for n in sm.MODEL_BS_LIST:
        if n in ks:
            ks[n].value = float(vals.get(n, 0.0))
    bpy.context.view_layer.update()


def pose_snapshot():
    bpy.context.view_layer.update()
    out = {}
    for pb in arm.pose.bones:
        out[pb.name] = (Vector(pb.head), Vector(pb.tail))
    return out


def mold_verts_snapshot(group_name, max_n=300):
    """mold 上某顶点组权重>0.2 的顶点（世界坐标），经评估后网格。"""
    if mold is None:
        return {}
    vg = mold.vertex_groups.get(group_name)
    if vg is None:
        return {}
    idx = [v.index for v in mold.data.vertices
           if any(g.group == vg.index and g.weight > 0.2 for g in v.groups)]
    idx = idx[:max_n]
    dg = bpy.context.evaluated_depsgraph_get()
    ev = mold.evaluated_get(dg)
    me = ev.to_mesh()
    mw = ev.matrix_world
    pts = {i: mw @ me.vertices[i].co for i in idx}
    ev.to_mesh_clear()
    return pts


INJECT = {
    "jawOpen": 1.0, "mouthSmileLeft": 1.0, "mouthUpperUpLeft": 1.0,
    "browInnerUp": 1.0, "browOuterUpLeft": 1.0, "eyeBlinkLeft": 1.0,
    "eyeLookUpLeft": 1.0, "noseSneerLeft": 1.0, "cheekPuff": 1.0,
    "tongueOut": 1.0, "mouthLowerDownLeft": 1.0,
}

set_arkit({})
bridge.apply_frame()
base_pose = pose_snapshot()
base_mold = {sid: mold_verts_snapshot(PREFIX + "%02d" % sid)
             for sid in (1, 2, 5, 9, 16, 21, 23)}

set_arkit(INJECT)
pwm = bridge.apply_frame()
act_pose = pose_snapshot()
act_mold = {sid: mold_verts_snapshot(PREFIX + "%02d" % sid)
            for sid in base_mold}
eye_rot = {}
for en in ("eyes.L", "eyes.R"):
    ob = bpy.data.objects.get(en)
    if ob is not None:
        eye_rot[en] = [round(math.degrees(v), 3) for v in ob.rotation_euler]

motion_rows = []
for pb in sorted(arm.pose.bones, key=lambda x: x.name):
    sid = int(pb.name.rsplit("_", 1)[1])
    h0, t0 = base_pose[pb.name]
    h1, t1 = act_pose[pb.name]
    rest_dir = (t0 - h0)
    pose_dir = (t1 - h1)
    axis = rest_dir.cross(pose_dir)
    angle = rest_dir.angle(pose_dir) if rest_dir.length > 1e-9 and pose_dir.length > 1e-9 else 0.0
    # 旋转轴（面坐标系）：rest 矩阵局部 X
    b = arm.data.bones[pb.name]
    local_x_arm = b.matrix_local.to_3x3() @ Vector((1.0, 0.0, 0.0))
    lx_face = Vector((local_x_arm[0] / SX, local_x_arm[1] / SY, local_x_arm[2] / SZ)).normalized()
    ch = chains.get(str(sid)) or {}
    servo = ch.get("servo_t_mm")
    head = Vector(b.head_local)
    pull_face = None
    if servo:
        pull_b = (cadworld_to_blender(servo) - head).normalized()
        pull_face = Vector((pull_b[0] / SX, pull_b[1] / SY, pull_b[2] / SZ)).normalized()
    motion_rows.append({
        "id": sid,
        "activation": round(float(bpy.context.scene["morpheus_activation"][sid - 1]), 4),
        "head_disp_mm": round((h1 - h0).length * MM_PER_UNIT, 4),
        "tail_disp_mm": round((t1 - t0).length * MM_PER_UNIT, 3),
        "rot_deg": round(math.degrees(angle), 3),
        "rot_axis_face": [round(v, 3) for v in lx_face],
        "pull_dir_face": [round(v, 3) for v in pull_face] if pull_face else None,
    })

# 蒙皮顶点轨迹：平均位移方向 vs 「拉向舵机」方向 / 下颌向下
skin_rows = []
for sid, base in base_mold.items():
    act = act_mold.get(sid, {})
    common = [i for i in base if i in act]
    if not common:
        skin_rows.append({"id": sid, "verts": 0})
        continue
    disp = Vector((0.0, 0.0, 0.0))
    for i in common:
        disp += act[i] - base[i]
    disp /= len(common)
    disp_face = Vector((disp[0] / SX, disp[1] / SY, disp[2] / SZ))
    mag = disp_face.length
    d = disp_face.normalized() if mag > 1e-9 else Vector((0, 0, 0))
    ch = chains.get(str(sid)) or {}
    servo = ch.get("servo_t_mm")
    pull_face = None
    if servo:
        head = Vector(arm.data.bones[PREFIX + "%02d" % sid].head_local)
        pb_ = (cadworld_to_blender(servo) - head).normalized()
        pull_face = Vector((pb_[0] / SX, pb_[1] / SY, pb_[2] / SZ)).normalized()
    skin_rows.append({
        "id": sid, "verts": len(common),
        "mean_disp_mm": round(mag, 3),
        "disp_dir_face": [round(v, 3) for v in d],
        "angle_to_pull_dir_deg": ang_deg(d, pull_face) if pull_face else None,
        "angle_to_jaw_down_deg": ang_deg(d, Vector((0.0, 0.3, -1.0)).normalized()) if sid == 23 else None,
    })

# 恢复中立（不保存）
set_arkit({})
bridge.apply_frame()

report = {
    "file": bpy.data.filepath,
    "calibration": calib,
    "scene": scene_info,
    "static": static_rows,
    "motion": motion_rows,
    "skin_trajectory": skin_rows,
    "eyes_rotation_deg": eye_rot,
    "pwm_after_inject": pwm,
}
(ROOT / "mapping_audit.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")

# --------------------------------------------------------------- 摘要
print("=== SCENE ===")
print(json.dumps(scene_info, ensure_ascii=False))
print("=== STATIC (head 位置核对) ===")
ds, dd = [], []
for r in static_rows:
    if r["dist_head_to_servo_axis_mm"] is not None:
        ds.append(r["dist_head_to_servo_axis_mm"])
    if r["dist_head_to_cad_driver_mm"] is not None:
        dd.append(r["dist_head_to_cad_driver_mm"])
    print("M%02d head->servo=%7s mm  head->cad_driver=%7s mm  tail->servo=%6s deg  deform=%s  %s" % (
        r["id"], r["dist_head_to_servo_axis_mm"], r["dist_head_to_cad_driver_mm"],
        r["tail_to_servo_angle_deg"], r["use_deform"], r["cad_driver_part"] or "-"))
print("mean head->servo axis = %.1f mm (n=%d); mean head->cad driver = %.1f mm (n=%d)" % (
    sum(ds) / len(ds), len(ds), sum(dd) / len(dd), len(dd)))
print("=== MOTION (注入 11 个 ARKit=1 后) ===")
for r in motion_rows:
    if r["rot_deg"] > 0.01 or r["head_disp_mm"] > 0.01:
        print("M%02d act=%+.2f head_disp=%.3fmm tail_disp=%.2fmm rot=%.2fdeg axis_face=%s" % (
            r["id"], r["activation"], r["head_disp_mm"], r["tail_disp_mm"],
            r["rot_deg"], r["rot_axis_face"]))
print("=== SKIN TRAJECTORY (mold 顶点平均位移) ===")
for r in skin_rows:
    print(json.dumps(r, ensure_ascii=False))
print("=== EYES (注入后 L/R 旋转 deg) ===")
print(json.dumps(eye_rot, ensure_ascii=False))
print("AUDIT_DONE")