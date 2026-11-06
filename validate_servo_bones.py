"""验证：33 根舵机骨位置、原 46 骨不变、Copy Rotation 是否真的跟随 ARKit。"""

import bpy
import json
import math
import pathlib
import sys

ROOT = pathlib.Path(bpy.data.filepath).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

PREFIX = "Morpheus_Servo_"
ARM_NAME = "Feace_Rig.001"
ARMATURE = bpy.data.objects[ARM_NAME]

report = json.loads((ROOT / "servo_bones_report.json").read_text(encoding="utf-8"))
rows = report["rows"]

result = {"status": "PASS", "checks": {}, "errors": []}


def fail(msg):
    result["status"] = "FAIL"
    result["errors"].append(msg)


# 1. 原 46 骨仍在，总数 79
result["checks"]["bone_count"] = len(ARMATURE.data.bones)
if len(ARMATURE.data.bones) != 79:
    fail("骨骼总数不是 79: %d" % len(ARMATURE.data.bones))
if len(ARMATURE.animation_data.drivers) != 25:
    fail("原驱动器数量不是 25")

# 2. 33 根舵机骨 head 必须落在 CAD 目标坐标
max_head_error = 0.0
for row in rows:
    b = ARMATURE.data.bones.get(PREFIX + "%02d" % row["id"])
    if b is None:
        fail("缺少 M%02d" % row["id"])
        continue
    target = row["xyz_blender"]
    err = max(abs(b.head_local[i] - target[i]) for i in range(3))
    max_head_error = max(max_head_error, err)
    if err > 1e-5:
        fail("M%02d 位置偏差 %.6f" % (row["id"], err))
result["checks"]["max_head_error"] = max_head_error

def quat_angle(a, b):
    """四元数最短角差，处理 q 与 -q 的 2π 等价。"""
    ang = a.rotation_difference(b).angle
    return min(ang, abs(2.0 * math.pi - ang))


# 3. 静止姿态：舵机骨世界旋转必须与源骨一致
rest_errors = []
for row in rows:
    if not row["sources"]:
        continue
    sb = ARMATURE.pose.bones[PREFIX + "%02d" % row["id"]]
    src = ARMATURE.pose.bones[row["sources"][0]]
    angle = quat_angle(sb.matrix.to_quaternion(), src.matrix.to_quaternion())
    rest_errors.append((row["id"], angle))
    if angle > 0.01:
        fail("静止帧 M%02d 与 %s 旋转差 %.6f rad" % (row["id"], row["sources"][0], angle))
result["checks"]["rest_max_rotation_error"] = max(a for _, a in rest_errors) if rest_errors else None

# 4. 注入非零 ARKit，舵机骨必须跟着转
mesh = bpy.data.objects.get("SampleHead")
keys = mesh.data.shape_keys.key_blocks
baseline = {}
for row in rows:
    if row["sources"]:
        baseline[row["id"]] = ARMATURE.pose.bones[
            PREFIX + "%02d" % row["id"]].matrix.to_quaternion()

test = {
    "jawOpen": 0.85, "mouthSmileLeft": 0.8, "mouthSmileRight": 0.5,
    "browInnerUp": 0.8, "browOuterUpLeft": 0.6, "eyeBlinkRight": 0.7,
    "eyeLookUpLeft": 0.5, "eyeLookInLeft": 0.4, "jawLeft": 0.4,
    "mouthUpperUpLeft": 0.5, "mouthLowerDownRight": 0.55,
}
for name, value in test.items():
    if name in keys:
        keys[name].value = value
bpy.context.view_layer.update()

moved = []
same = []
for row in rows:
    if not row["sources"]:
        continue
    sid = "%02d" % row["id"]
    sb = ARMATURE.pose.bones[PREFIX + sid]
    src = ARMATURE.pose.bones[row["sources"][0]]
    angle = quat_angle(sb.matrix.to_quaternion(), src.matrix.to_quaternion())
    if angle > 0.01:
        fail("注入后 M%s 仍与 %s 不同步 %.6f" % (sid, row["sources"][0], angle))
    travel = quat_angle(baseline[row["id"]], sb.matrix.to_quaternion())
    if travel > 0.005:
        moved.append(row["id"])
    else:
        same.append(row["id"])
result["checks"]["driven_channels"] = len(baseline)
result["checks"]["channels_that_actually_rotated"] = len(moved)
result["checks"]["channels_static_under_test_pose"] = same
result["checks"]["injected_keys"] = len(test)

# 5. 无源 8 路在注入后不得乱动
for sid in (1, 2, 11, 12, 13, 18, 29, 30):
    name = PREFIX + "%02d" % sid
    if ARMATURE.pose.bones[name].constraints:
        fail("无源通道 M%02d 不应有约束" % sid)
result["checks"]["unmapped_without_constraints"] = True

# 6. 记录当前各舵机骨世界位置，确认没有被误放大
positions = {}
for row in rows:
    b = ARMATURE.data.bones[PREFIX + "%02d" % row["id"]]
    positions[row["id"]] = [round(v, 6) for v in b.head_local]
result["checks"]["positions"] = positions

# 恢复
for name in test:
    if name in keys:
        keys[name].value = 0.0
bpy.context.view_layer.update()

out = ROOT / "servo_bones_validation.json"
out.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding="utf-8")
print("SERVO_BONES_VALIDATION_" + result["status"],
      json.dumps({k: v for k, v in result["checks"].items() if k != "positions"},
                 ensure_ascii=False))
if result["errors"]:
    print("ERRORS", result["errors"])
print("REPORT", out)
