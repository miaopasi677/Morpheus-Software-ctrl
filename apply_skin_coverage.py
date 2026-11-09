"""补齐 33 路蒙皮并修 M31过载；眼球拆 L/R 独立枢轴（由桥接按 M07/M24 正向转角驱动）。

规则：
  * 缺失顶点组：以 spec.anchor（CAD 表皮驱动点 / 蒙皮代理点）为中心，面坐标距离
    余弦衰减加权；已有组不覆盖，顶点总权重 >1 时归一化。
  * M31（颈旋转）原覆盖 90% 模具 -> 稀释全部表情。面部顶点（face Z > -100mm）
    移除 M31 权重；颈区保留。整头随颈转动是场景级变换，不在模具蒙皮内做。
  * M32/M33（颈点头左/右）：颈区（Z <= -100mm）按左右侧衰减加组。
  * eyes 网格两个连通球 -> eyes.L / eyes.R，各自原点=球心，原 eyes 隐藏保留引用。

运行：
  blender --background LY_GENERAL_V0.blend --python apply_skin_coverage.py \
          --python-expr "import bpy; bpy.ops.wm.save_mainfile()"
"""
import json
import math
import pathlib
import sys

import bmesh
import bpy
from mathutils import Vector

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import mechanism_kinematics as mk

PREFIX = "Morpheus_Servo_"
drv = json.loads((ROOT / "driver_bones_report.json").read_text(encoding="utf-8"))
c = drv["calibration"]
EM = Vector(c["eye_mid"])
SX, SY, SZ = c["sx"], c["sy"], c["sz"]


def to_face(p):
    return Vector(((p[0] - EM[0]) / SX, (p[1] - EM[1]) / SY, (p[2] - EM[2]) / SZ))


def falloff(d, r):
    if d >= r:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * d / r))


# 每路作用半径（face mm）：口角/下颌大，鼻/颊小
RADIUS = {1: 20, 2: 15, 11: 22, 12: 18, 13: 20, 14: 22, 18: 25, 19: 22,
          29: 20, 30: 20}
NECK_Z = -100.0


def split_eyes():
    eyes = bpy.data.objects.get("eyes")
    if eyes is None or bpy.data.objects.get("eyes.L"):
        return None
    bm = bmesh.new()
    bm.from_mesh(eyes.data)
    bm.verts.ensure_lookup_table()
    seen, comps = set(), []
    for v in bm.verts:
        if v.index in seen:
            continue
        stack, comp = [v.index], []
        while stack:
            ui = stack.pop()
            if ui in seen:
                continue
            seen.add(ui)
            comp.append(ui)
            u = bm.verts[ui]
            stack.extend(e.other_vert(u).index for e in u.link_edges)
        comps.append(comp)
    bm.free()
    if len(comps) != 2:
        return None
    mw = eyes.matrix_world
    centers = []
    for comp in comps:
        cen = sum((mw @ eyes.data.vertices[i].co for i in comp), Vector()) / len(comp)
        centers.append(cen)
    # 面坐标 X 正 = 人物左
    order = sorted(range(2), key=lambda i: -to_face(centers[i])[0])
    info = []
    for tag, ci in zip(("L", "R"), order):
        comp = comps[ci]
        me = bpy.data.meshes.new("eyes_%s_mesh" % tag)
        vids = sorted(comp)
        remap = {old: new for new, old in enumerate(vids)}
        me.from_pydata([eyes.data.vertices[i].co[:] for i in vids],
                       [(remap[e.vertices[0]], remap[e.vertices[1]])
                        for e in eyes.data.edges if e.vertices[0] in remap and e.vertices[1] in remap],
                       [[remap[i] for i in p.vertices if i in remap]
                        for p in eyes.data.polygons if all(i in remap for i in p.vertices)])
        me.update()
        ob = bpy.data.objects.new("eyes.%s" % tag, me)
        bpy.context.collection.objects.link(ob)
        ob.matrix_world = eyes.matrix_world.copy()
        # 原点移到球心（保持世界位置不变）
        cen_local = eyes.matrix_world.inverted() @ centers[ci]
        for v in me.vertices:
            v.co -= cen_local
        ob.location = eyes.matrix_world @ cen_local
        info.append({"name": ob.name, "verts": len(vids),
                     "center_world": [round(v, 5) for v in centers[ci]]})
    eyes.hide_render = True
    eyes.hide_viewport = True
    return info


def main():
    mold = bpy.data.objects["mold"]
    mw = mold.matrix_world
    face = [to_face(mw @ v.co) for v in mold.data.vertices]
    groups = {g.name: g for g in mold.vertex_groups}
    report = {"eyes": split_eyes(), "added": {}, "m31_trimmed": 0}

    # ---- 缺失面部组
    for sid, r_mm in RADIUS.items():
        name = PREFIX + "%02d" % sid
        spec = mk.build_spec(sid)
        center = spec.get("anchor")
        if center is None:
            report["added"][name] = {"skipped": "no anchor"}
            continue
        cf = to_face(center)
        vg = groups.get(name) or mold.vertex_groups.new(name=name)
        n = 0
        for i, v in enumerate(mold.data.vertices):
            w = falloff((face[i] - cf).length, r_mm)
            if w > 0.02:
                vg.add([i], min(w, 0.85), "REPLACE")
                n += 1
        report["added"][name] = {"verts": n, "center_face_mm": [round(x, 1) for x in cf],
                                 "radius_mm": r_mm, "confidence": spec["confidence"]}

    # ---- 颈区组 M32/M33（左右差分点头）
    neck_idx = [i for i, f in enumerate(face) if f[2] <= NECK_Z]
    for sid, side in ((32, 1.0), (33, -1.0)):
        name = PREFIX + "%02d" % sid
        vg = groups.get(name) or mold.vertex_groups.new(name=name)
        n = 0
        for i in neck_idx:
            f = face[i]
            d = ((f[0] - side * 39.0) ** 2 + (f[1] - 93.0) ** 2 + (f[2] + 204.0) ** 2) ** 0.5
            w = falloff(d, 90.0)
            if w > 0.02:
                vg.add([i], min(w, 0.8), "REPLACE")
                n += 1
        report["added"][name] = {"verts": n, "region": "neck Z<=-100mm", "side": side}

    # ---- M31 过载修剪：面部顶点移除颈旋转权重
    vg31 = groups.get(PREFIX + "31")
    if vg31 is not None:
        rm = [i for i, f in enumerate(face) if f[2] > NECK_Z
              and any(g.group == vg31.index and g.weight > 0 for g in mold.data.vertices[i].groups)]
        if rm:
            vg31.remove(rm)
        report["m31_trimmed"] = len(rm)

    # ---- 归一化：总权重 >1 的顶点按比例缩到 1
    servo_idx = {g.index for g in mold.vertex_groups if g.name.startswith(PREFIX)}
    over = 0
    for v in mold.data.vertices:
        ws = [(g.group, g.weight) for g in v.groups if g.group in servo_idx]
        tot = sum(w for _, w in ws)
        if tot > 1.0:
            for gi, w in ws:
                mold.vertex_groups[gi].add([v.index], w / tot, "REPLACE")
            over += 1
    report["normalized_verts"] = over
    report["mold_verts"] = len(mold.data.vertices)
    report["groups_total"] = len([g for g in mold.vertex_groups if g.name.startswith(PREFIX)])

    (ROOT / "skin_coverage_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    print("SKIN_COVERAGE_OK")


if __name__ == "__main__":
    main()