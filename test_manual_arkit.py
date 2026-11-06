import bpy
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(r"C:\file\project\Morpheus-Software")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import blender_servo_bridge as bridge

mesh = bpy.data.objects["SampleHead"]
keys = mesh.data.shape_keys
# 只在本次后台测试会话暂时摘掉全 0 动作，不保存这个改动。
keys.animation_data.action = None
bpy.context.scene.frame_set(1)
values = {k.name: 0.0 for k in keys.key_blocks}
for name, value in {
    "browInnerUp": 0.75,
    "browOuterUpLeft": 0.55,
    "eyeBlinkRight": 0.65,
    "eyeLookUpLeft": 0.45,
    "jawOpen": 0.80,
    "jawLeft": 0.35,
    "mouthSmileLeft": 0.75,
    "mouthSmileRight": 0.45,
    "mouthUpperUpLeft": 0.40,
    "mouthLowerDownRight": 0.50,
}.items():
    if name in keys.key_blocks:
        keys.key_blocks[name].value = value
mesh.update_tag()
bpy.context.view_layer.update()
depsgraph = bpy.context.evaluated_depsgraph_get()

report = {"arkit": {}, "targets": {}, "morpheus": bridge.morpheus_details()}
for name, value in values.items():
    if value:
        report["arkit"][name] = value
for name in ("browInnerE.R", "browOuterE.L", "eyeE.L", "eyeLidUpE.R",
             "lipUpE.L", "lipDownE.R", "lipConnerE.L", "lipConnerE.R", "jawE"):
    obj = bpy.data.objects[name]
    evaluated = obj.evaluated_get(depsgraph)
    vi = obj.parent_vertices[0]
    report["targets"][name] = {
        "world": list(evaluated.matrix_world.translation),
        "local": list(obj.location),
        "vertex": list(mesh.evaluated_get(depsgraph).data.vertices[vi].co),
    }

base = json.loads((ROOT / "rig_audit.json").read_text(encoding="utf-8"))
base_targets = {o["name"]: o for o in base["objects"]}
travel = {}
for name, row in report["targets"].items():
    # 原 rig_audit 的 location 是局部位置，只用于粗略诊断；世界坐标打印供直接比较。
    travel[name] = row["world"]
print("MANUAL_ARKIT", json.dumps(report["arkit"], ensure_ascii=False))
print("TARGETS", json.dumps(report["targets"], ensure_ascii=False))
print("PWM", [r["pwm"] for r in report["morpheus"]])
print("SOURCES", [(r["id"], r["source_bone"], r["legacy_position"], r["pwm"]) for r in report["morpheus"] if r["mapped"]])
(ROOT / "manual_arkit_diagnostic.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
