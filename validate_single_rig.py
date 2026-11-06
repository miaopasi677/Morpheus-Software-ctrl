"""验证单骨架重建：33 骨、ARKit 驱动、PWM、蒙皮形变。"""

import json
import math
import pathlib
import sys

import bpy

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import servo_map as sm
import morpheus_arkit_bridge as bridge

ARM_NAME = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"
REPORT = ROOT / "single_rig_validation.json"

R = {"status": "pass", "checks": {}, "failures": []}


def fail(msg):
    R["status"] = "fail"
    R["failures"].append(msg)


def mesh_coords(name):
    obj = bpy.data.objects.get(name)
    if obj is None:
        return None
    # 形态键 / 骨骼写入后必须强制求值，否则拿到的是上一次缓存
    bpy.context.view_layer.update()
    dg = bpy.context.evaluated_depsgraph_get()
    dg.update()
    ev = obj.evaluated_get(dg)
    m = ev.to_mesh()
    pts = [tuple(v.co) for v in m.vertices]
    ev.to_mesh_clear()
    return pts


def main():
    arm = bpy.data.objects[ARM_NAME]
    bones = [b.name for b in arm.data.bones]
    R["checks"]["bones"] = len(bones)
    R["checks"]["all_morpheus"] = all(n.startswith(PREFIX) for n in bones)
    if len(bones) != 33 or not R["checks"]["all_morpheus"]:
        fail("骨架不是纯 33 根 Morpheus 骨")

    ad = arm.animation_data
    R["checks"]["deform_bones"] = sum(1 for b in arm.data.bones if b.use_deform)
    if R["checks"]["deform_bones"] != 33:
        fail("只有 %d 根骨参与蒙皮" % R["checks"]["deform_bones"])
    R["checks"]["armature_drivers"] = len(ad.drivers) if ad else 0
    R["checks"]["constraints"] = sum(len(pb.constraints) for pb in arm.pose.bones)
    if R["checks"]["armature_drivers"]:
        fail("残留指向已删骨的驱动器")

    # 蒙皮
    mold = bpy.data.objects["mold"]
    groups = [g.name for g in mold.vertex_groups]
    R["checks"]["mold_groups"] = groups
    R["checks"]["mold_groups_ok"] = all(g.startswith(PREFIX) for g in groups)
    R["checks"]["eyes_armature_mods"] = sum(
        1 for m in bpy.data.objects["eyes"].modifiers if m.type == "ARMATURE")
    if not R["checks"]["mold_groups_ok"]:
        fail("mold 仍有旧顶点组")
    if R["checks"]["eyes_armature_mods"]:
        fail("eyes 仍挂失效的 Armature 修改器")

    # 文本块：只保留一份新桥接
    texts = [t.name for t in bpy.data.texts]
    R["checks"]["texts"] = texts
    if "morpheus_arkit_bridge.py" not in texts:
        fail("缺少新桥接文本块")
    if "morpheus_servo_bridge.py" in texts:
        fail("旧桥接文本块未删除")

    # ---------- 中立态 ----------
    ks = bpy.data.objects["SampleHead"].data.shape_keys.key_blocks
    for k in ks:
        if k.name != "Basis":
            k.value = 0.0
    neutral_pwm = bridge.apply_frame()
    neutral_rot = {i: arm.pose.bones[PREFIX + "%02d" % i].rotation_euler[0]
                   for i in range(1, 34)}
    R["checks"]["neutral_pwm"] = neutral_pwm
    R["checks"]["neutral_rotation_max"] = max(abs(v) for v in neutral_rot.values())

    # ---------- 注入非零 ARKit ----------
    test = {
        "jawOpen": 0.85, "mouthSmileLeft": 0.80, "mouthSmileRight": 0.45,
        "mouthUpperUpLeft": 0.60, "mouthLowerDownRight": 0.55,
        "browInnerUp": 0.85, "browOuterUpLeft": 0.70,
        "eyeBlinkLeft": 0.75, "eyeBlinkRight": 0.65,
        "eyeLookUpLeft": 0.50, "eyeLookInLeft": 0.40,
        "mouthPucker": 0.35, "noseSneerLeft": 0.45,
        "cheekPuff": 0.40, "tongueOut": 0.55,
    }
    applied = 0
    for k, v in test.items():
        if k in ks:
            ks[k].value = v
            applied += 1
    R["checks"]["injected_keys"] = applied
    if applied < 12:
        fail("注入的形态键不足 %d" % applied)

    pwm = bridge.apply_frame()

    rotated, changed = [], []
    for i in range(1, 34):
        r = arm.pose.bones[PREFIX + "%02d" % i].rotation_euler[0]
        if abs(r - neutral_rot[i]) > 1e-5:
            rotated.append((i, round(r, 4)))
        if pwm[i - 1] != neutral_pwm[i - 1]:
            changed.append(i)
    R["checks"]["bones_rotated"] = len(rotated)
    R["checks"]["pwm_changed_channels"] = changed
    R["checks"]["active_pwm"] = pwm
    R["checks"]["rotation_detail"] = rotated
    if len(rotated) < 15:
        fail("仅 %d 根骨响应 ARKit" % len(rotated))
    if len(changed) < 15:
        fail("仅 %d 路 PWM 变化" % len(changed))

    # PWM 限幅
    bad = []
    for i, v in enumerate(pwm, 1):
        s = sm.BY_ID[i]
        if not (s["pwm_low"] <= v <= s["pwm_high"]):
            bad.append((i, v, s["pwm_low"], s["pwm_high"]))
    R["checks"]["pwm_out_of_range"] = bad
    if bad:
        fail("PWM 越界 %s" % bad[:3])

    # Scene 属性
    sc = bpy.context.scene
    R["checks"]["scene_pwm_len"] = len(sc["morpheus_pwm"])
    R["checks"]["scene_activation_len"] = len(sc["morpheus_activation"])
    R["checks"]["scene_arkit_len"] = len(sc["morpheus_arkit"])
    if len(sc["morpheus_pwm"]) != 33:
        fail("scene morpheus_pwm 不是 33 路")

    # 形变：evaluated mesh 与基础网格比较，证明形态键 / 骨架修改器确实在求值
    bpy.context.view_layer.update()
    head_eval = mesh_coords("SampleHead")
    mold_eval = mesh_coords("mold")
    head_base = [tuple(v.co) for v in bpy.data.objects["SampleHead"].data.vertices]
    mold_base = [tuple(v.co) for v in mold.data.vertices]
    hd = max(math.dist(a, b) for a, b in zip(head_base, head_eval))
    md = max(math.dist(a, b) for a, b in zip(mold_base, mold_eval))
    R["checks"]["samplehead_shapekey_delta"] = round(hd, 5)
    R["checks"]["mold_armature_delta"] = round(md, 5)
    if hd > 100.0 or md > 100.0:
        fail("网格形变异常 head=%.3f mold=%.3f" % (hd, md))
    if hd < 1e-4:
        fail("SampleHead 形态键未产生形变")

    REPORT.write_text(json.dumps(R, ensure_ascii=False, indent=1), encoding="utf-8")
    print("STATUS", R["status"])
    print("CHECKS", json.dumps({k: v for k, v in R["checks"].items()
                                if k not in ("mold_groups", "texts", "rotation_detail",
                                             "neutral_pwm", "active_pwm")},
                               ensure_ascii=False)[:1500])
    for f in R["failures"]:
        print("FAIL", f)


if __name__ == "__main__":
    main()
