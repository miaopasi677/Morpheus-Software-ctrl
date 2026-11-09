"""把 33 根 Morpheus_Servo 骨摆到 CAD 真实几何的 rest 位置（建准模型）。

rest 位置规则（mechanism_kinematics.build_spec）：
  translate  head=表皮驱动点 anchor，tail 沿真实驱动方向
  fourbar    head=摇杆真实固定铰 D，tail 指向摇杆销 C0（绕 D 摆=真实轨迹）
  revolute   head=真实铰链（眼球中心/下颌球铰/颈平台），tail 指向 anchor

蒙皮不变（armature 以 rest 为 bind，中立位不跳变）；改 rest 只改变旋转/平动枢轴。
写入运动学属性供核对。备份后保存。

运行：
  blender --background LY_GENERAL_V0.blend --python apply_mechanism_rig.py \
          --python-expr "import bpy; bpy.ops.wm.save_mainfile()"
"""
import json
import pathlib
import shutil
import sys

import bpy
from mathutils import Vector

ROOT = pathlib.Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import mechanism_kinematics as mk

ARM = "Feace_Rig.001"
PREFIX = "Morpheus_Servo_"


def main():
    arm = bpy.data.objects[ARM]
    bak = ROOT / "LY_GENERAL_V0_before_mechanism_rig.blend"
    if not bak.exists():
        shutil.copy2(ROOT / "LY_GENERAL_V0.blend", bak)

    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    if arm.mode != "EDIT":
        bpy.ops.object.mode_set(mode="EDIT")
    edit = arm.data.edit_bones
    rows = []
    for sid in range(1, 34):
        name = PREFIX + "%02d" % sid
        eb = edit.get(name)
        if eb is None:
            continue
        spec = mk.build_spec(sid)
        head = spec.get("rest_head") or spec.get("anchor") or Vector(eb.head)
        tail = spec.get("rest_tail")
        if tail is None or (tail - head).length < 1e-5:
            tail = head + Vector((0, 0.12, 0))
        eb.head = head
        eb.tail = tail
        eb.roll = 0.0
        eb.use_deform = True
        rows.append({"id": sid, "mode": spec["mode"], "type": spec["type"],
                     "confidence": spec["confidence"],
                     "head": [round(v, 4) for v in head],
                     "tail": [round(v, 4) for v in tail]})
    bpy.ops.object.mode_set(mode="OBJECT")

    for r in rows:
        b = arm.data.bones[PREFIX + "%02d" % r["id"]]
        b["mech_mode"] = r["mode"]
        b["mech_type"] = r["type"]
        b["mech_confidence"] = r["confidence"]
        b["mapping"] = "ARKIT_TO_SERVO normalized -> mechanism_kinematics (forward, no IK)"

    report = {"armature": ARM, "bones": rows,
              "note": "rest=head at real pivot/anchor; skin bind unchanged; forward drive in bridge"}
    (ROOT / "mechanism_rig_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
    for r in rows:
        print("M%02d %-9s %-9s %s" % (r["id"], r["mode"], r["type"], r["confidence"]))
    print("MECHANISM_RIG_OK bones=%d" % len(rows))


if __name__ == "__main__":
    main()