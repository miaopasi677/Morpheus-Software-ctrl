"""
Morpheus 33 路舵机的几何：物理毫米坐标 + 到 LY_GENERAL_V0.blend 的配准。

【2026-11-04 更新】用户已把 SolidWorks 源文件放到 CAD_model/，坐标不再是估计。

数据来源优先级：
  1. CAD_model/full_assembly.SLDASM（SolidWorks 2022，权威）
     经开源解码器 sldprt 读出 swXmlContents/COMPINSTANCETREE.xml，
     得到 280 个组件实例的 4x4 变换矩阵（swTransform，单位 m）与
     128 个模型的 swBoundingBox。其中 32 个是实体舵机实例。
     解析链：cad_geometry/extract_asm_stream.js -> parse_cad_assembly.py
             -> build_cad_geometry.py -> cad_geometry/servo_id_mapping.json
     本文件在导入时自动加载该 JSON，覆盖 _ROWS 里的估计坐标。
  2. CAD_model/*.SLDPRT -> cad_geometry/stl/*.stl（121 个零件）
     精确外形尺寸见 cad_geometry/part_dimensions.json。
  3. Morpheus-Hardware README / 68 张装配图 / arXiv Fig.3、Fig.7
     用于解释模块层序、曲柄长度、线驱走线（见 HARDWARE_FACTS）。
  4. arXiv:2507.16645v1 Table V —— 33 路 PWM 限位（见 servo_map.py）。

精度声明：
  - cad_measured（32 路）：坐标来自 CAD 装配变换矩阵，毫米级精确；
    但「哪一路 PWM 对应哪一个实体舵机」仍是按模块+层序+左右做的推断，
    见 servo_id_mapping.json 的 shape_distance_mm（越大越不确定）。
  - estimated_no_cad_instance（M31 颈中央主轴）：CAD 里没有独立舵机实例，
    沿用 _ROWS 估计值。
  - _ROWS 现在只作为回退 / 语义说明（mount、fig7_rank、note 仍来自它）。

坐标系（face，物理 mm，原点 = 两眼球中点）：
  +X = 人物自身左（与 .blend 的 +X 一致）；+Y = 脑后（脸朝 -Y）；
  +Z = 头顶。
CAD 装配世界坐标 -> face 的刚体变换（由 eye_camera_eyeball 与
shell_left/right_ears 定标，见 build_cad_geometry.py）：
  face_x = Z_world - 1.0 ; face_y = X_world + 96.84 ; face_z = Y_world - 285.87
"""

from __future__ import annotations

import json
import pathlib

# ---------------------------------------------------------------- 精确事实
HARDWARE_FACTS = {
    "studs_mm": {
        "neck_to_mouth": {"count": 4, "length": 60, "image": "assembly_2.jpg"},
        "mouth_to_eye": {"count": 4, "length": 30, "image": "assembly_4.jpg"},
        "eye_to_eyebrow": {"count": 3, "length": 60, "image": "assembly_6.jpg"},
    },
    "crank_mm": {"jaw": 32, "ks3518_jaw": 38, "pushrod": 35,
                 "lip_wire_diameter": 2, "linear_rail": 55},
    "servo_models": {"GUOHUA_A0090_9g": 29, "KS3518": 4,
                     "GUOHUA_A0090_body_mm": [22.5, 12.0, 22.0]},
    "control": {"pwm_range": [0, 2000], "servo_hz": 50,
                "virtual_fps": 30, "board": "Raspberry Pi 5 / Jetson AGX Xavier"},
    "counts": {"total": 33, "rigid": 29, "tendon": 4,
               "by_module": {"eyebrow": 4, "eye": 6, "mouth": 16, "neck": 3},
               "inverse_channels": 26},
    # ---- 以下为 CAD_model/ 的 SolidWorks 2022 实测值（替换了上面的估计） ----
    "cad_measured": {
        "saved_with": "SolidWorks 2022 (version 15000)",
        "component_instances": 280,
        "servo_instances": 32,
        "servo_by_module": {"eyebrow": 8, "eye": 6, "mouth": 16, "neck": 2},
        "eye_midpoint_world_mm": [-96.84, 285.87, 1.0],
        "eye_span_mm": 70.0,
        "shell_left_ear_world_z_mm": 82.55,
        "shell_right_ear_world_z_mm": -80.013,
        "subassembly_origin_world_mm": {
            "neck": [0.0, 0.0, 0.0], "mouth": [-53.0, 220.5, 1.0],
            "eye": [-58.84, 256.87, 1.0], "eyebrow": [-51.84, 311.87, 1.0]},
        "studs_world_y_mm": {"60mm_bottom": 160.5, "60mm_top": 220.5,
                             "30mm_bottom": 223.5, "30mm_top": 253.5,
                             "60mm_eye_bottom": 259.87, "60mm_eye_top": 319.87},
        "servo_body_size_mm": {
            "MG90S": [32.1, 32.5, 12.403],
            "GUOHUAA0090": [32.1, 31.9, 12.203],
            "KS3518": [54.0, 45.3, 18.0]},
        "structural_size_mm": {
            "neck_Parallel_platforms": [60.0, 5.0, 60.0],
            "mouth_baseplate": [128.44, 49.75, 129.0],
            "eye_mounted_version": [130.0, 41.7, 93.986],
            "eyebrow_Installation platform": [94.0, 41.5, 83.0]},
        "servo_face_z_range_mm": {
            "neck": [-215.4, -215.4], "mouth": [-105.4, -46.6],
            "eye": [-34.2, 2.6], "eyebrow": [36.1, 60.5]},
        "cad_to_face": "face_x=Z_world-1, face_y=X_world+96.84, face_z=Y_world-285.87",
    },
    "sources": [
        "CAD_model/full_assembly.SLDASM (SolidWorks 2022, authoritative)",
        "cad_geometry/COMPINSTANCETREE.xml (280 component transforms)",
        "cad_geometry/servo_id_mapping.json (32 measured servo positions)",
        "cad_geometry/part_dimensions.json (121 exact part sizes)",
        "hardware_ref/README.md",
        "hardware_ref/media/*.jpg (68 images)",
        "hardware_ref/sup_motor.png  (arXiv Fig.7, channel rank order)",
        "hardware_ref/mechanial_face.png (arXiv Fig.3)",
        "arXiv:2507.16645v1 Table V (PWM bounds, see servo_map.py)",
    ],
}

# ------------------------------------------------ 皮肤锚点（用于 mm -> Blender 配准）
# 物理毫米位置按成人面部比例 + 装配图眼部尺寸给定；Blender 侧取 Targets 集合
# 里同名控制点的 o.location（与 bone.head_local 同一坐标系）。
ANCHORS_MM = {
    "eyeE.L":         (31.5, -12.0,   0.0),
    "eyeE.R":         (-31.5, -12.0,  0.0),
    "browInnerE.L":   (12.0, -16.0,  22.0),
    "browInnerE.R":   (-12.0, -16.0, 22.0),
    "browOuterE.L":   (42.0, -14.0,  25.0),
    "browOuterE.R":   (-42.0, -14.0, 25.0),
    "noseE.L":        (16.0, -30.0, -28.0),
    "noseE.R":        (-16.0, -30.0, -28.0),
    "cheekE.L":       (48.0, -22.0, -25.0),
    "cheekE.R":       (-48.0, -22.0, -25.0),
    "lipUpE.C":       (0.0,  -20.0, -52.0),
    "lipConnerE.L":   (26.0, -18.0, -58.0),
    "lipConnerE.R":   (-26.0, -18.0, -58.0),
    "lipDownE.C":     (0.0,  -18.0, -68.0),
    "jawE":           (0.0,  -16.0, -85.0),
}

# ------------------------------------------------------------ 33 路真实布局
# 字段: id, module, x, y, z (mm), side, fig7_rank, mount(CAD 零件), crank_mm,
#       axis(输出轴大致朝向), note
_ROWS = [
    # --- 线驱 4 路：装在眉模块顶部/头顶，线穿过眼/眉模块预留孔牵拉鼻与颊 ---
    (1,  "tendon",  52, 50,  88, "L",  1, "cheek_line.SLDPRT",              None, "Z", "左颊线驱大舵机，头顶"),
    (2,  "tendon",  30, 50,  78, "L",  2, "nose_line.SLDPRT",               None, "Z", "左鼻线驱大舵机，头顶"),
    (29, "tendon", -30, 50,  78, "R", 29, "nose_mirror_line.SLDPRT",        None, "Z", "右鼻线驱大舵机，头顶"),
    (30, "tendon", -52, 50,  88, "R", 30, "cheek_face_slave_part_mirror.SLDPRT", None, "Z", "右颊线驱大舵机，头顶"),

    # --- 眉 4 路：eyebrow_Installation_platform，背面交错 + 两侧中心支架 ---
    (3,  "eyebrow",  16, 42,  62, "L",  3, "eyebrow_brow center motor mounting bracket.SLDPRT", None, "X", "左眉头 四连杆"),
    (4,  "eyebrow",  46, 44,  58, "L",  4, "eyebrow_eyebrow peak rocker.SLDPRT",               None, "X", "左眉峰 四连杆"),
    (27, "eyebrow", -46, 44,  58, "R", 27, "eyebrow_mirror eyebrow peak rocker 1.SLDPRT",      None, "X", "右眉峰 四连杆"),
    (28, "eyebrow", -16, 42,  62, "R", 28, "eyebrow_mirror brow center motor mounting bracket.SLDPRT", None, "X", "右眉头 四连杆"),

    # --- 眼 6 路：4 眼睑在 eye_mounted_version 两侧，2 个 gaze 在两眼中线 ---
    (5,  "eye",  34, 28,  12, "L",  5, "eye_upper_eyelid_of_the_right_eye.SLDPRT", None, "X", "左上眼睑"),
    (6,  "eye",  32, 28,  -8, "L",  6, "eye_lower_eyelid_of_the_right_eye.SLDPRT", None, "X", "左下眼睑"),
    (25, "eye", -32, 28,  -8, "R", 25, "eye_lower_eyelid_of_the_right_eye_mirror.SLDPRT", None, "X", "右下眼睑"),
    (26, "eye", -34, 28,  12, "R", 26, "eye_upper_eyelid_of_the_right_eye_mirror.SLDPRT", None, "X", "右上眼睑"),
    (7,  "eye",   0, 36,   2, "C",  7, "eye_mounted_version.SLDPRT",          None, "Y", "眼球俯仰：两眼中线，eye_1.jpg"),
    (24, "eye",   0, 20,  -2, "C", 24, "eye_bassinet.SLDPRT / eye_comb.SLDPRT", None, "Y", "眼球左右：eye_bassinet，eye_2.jpg"),

    # --- 嘴 16 路：底板 / 六连杆 / 滑轨 / 舌四连杆 / 下唇底板 ---
    (8,  "mouth",   0, 32, -32, "C",  8, "mouth_baseplate.SLDPRT",                     None, "X", "上唇中央"),
    (9,  "mouth",  28, 30, -38, "L",  9, "mouth_motor mounting platform for smiling six-bar mechanism.SLDPRT", 32, "X", "左嘴角上 六连杆"),
    (10, "mouth", -18, 32, -33, "R", 10, "mouth_baseplate.SLDPRT",                     None, "X", "上唇右"),
    (21, "mouth",  18, 32, -33, "L", 21, "mouth_mounting platform for upper lip connecting rod.SLDPRT", None, "X", "上唇左"),
    (22, "mouth", -28, 30, -38, "R", 22, "mouth_mirror_motor mounting platform for smiling six-bar mechanism.SLDPRT", 32, "X", "右嘴角上 六连杆"),
    (23, "mouth",   0, 40, -46, "C", 23, "mouth_slide rail mounting plate.SLDPRT / mouth_crank.SLDPRT", 32, "X", "张合下颌，32mm 曲柄 + 五连杆"),
    (11, "mouth",   0, 26, -52, "C", 11, "mouth_tongue pulling rod.SLDPRT",           None, "X", "舌伸缩 四连杆"),
    (12, "mouth",   8, 26, -58, "L", 12, "mouth_tongue connecting rod_long.SLDPRT",  None, "X", "舌上下 四连杆"),
    (20, "mouth",  20, 38, -60, "L", 20, "mouth_jaw five-bar mechanism_long.SLDPRT", 38, "X", "下颌左右，38mm 曲柄 ks3518"),
    (13, "mouth", -10, 28, -76, "R", 13, "mouth_fourth-generation jaw.SLDPRT",       None, "X", "下巴 被动"),
    (18, "mouth",  10, 28, -76, "L", 18, "mouth_fourth-generation jaw.SLDPRT",       None, "X", "下巴 主动"),
    (14, "mouth",  28, 26, -66, "L", 14, "mouth_lip six-bar mechanism_8mm crossbar.SLDPRT", None, "X", "左嘴角下"),
    (19, "mouth", -28, 26, -66, "R", 19, "mouth_mirror_smiling five-bar mechanism_below.SLDPRT", None, "X", "右嘴角下"),
    (15, "mouth",   0, 26, -70, "C", 15, "mouth_baseplate for teeth.SLDPRT",         None, "X", "下唇中央"),
    (16, "mouth",  16, 26, -70, "L", 16, "mouth_baseplate for teeth.SLDPRT",         None, "X", "下唇左"),
    (17, "mouth", -16, 26, -70, "R", 17, "mouth_baseplate for teeth.SLDPRT",         None, "X", "下唇右"),

    # --- 颈 3 路：Fig.7 未画，按 neck_2.jpg 两舵机左右对称 + 中央主轴 ---
    (31, "neck",   0, 30, -120, "C", 31, "neck_principal_axis.SLDPRT / neck_universal_joint.SLDPRT", None, "Z", "颈旋转，中央主轴"),
    (32, "neck",  55, 30, -105, "L", 32, "neck_Installation_of_sheet_metal.SLDPRT (ks3518 left)",  35, "X", "颈点头/摇头 左"),
    (33, "neck", -55, 30, -105, "R", 33, "neck_Installation_of_sheet_metal.SLDPRT (ks3518 right)", 35, "X", "颈点头/摇头 右"),
]

# 物理原点在眼球中心；Fig.3(b) 透明侧视图显示整组舵机位于颅腔【后半部】。
# _ROWS 里的 y 是相对该原点的装配深度，统一加上此校正量后才是颅腔内位置。
# 这是唯一一个可调的全局深度参数：拿到 CAD 实测值后改这里（或逐行改 _ROWS）即可。
DEPTH_CORRECTION_MM = 30.0

SERVO_MM = {}
for (sid, module, x, y, z, side, rank, mount, crank, axis, note) in _ROWS:
    SERVO_MM[sid] = {
        "id": sid, "module": module, "xyz_mm": (float(x), float(y) + DEPTH_CORRECTION_MM, float(z)),
        "side": side, "fig7_rank": rank, "mount": mount,
        "crank_mm": crank, "axis": axis, "note": note,
        "vertical_order_source": "arXiv Fig.7 (sup_motor.png) numbered leader lines",
        "confidence": "reconstructed",
    }
assert len(SERVO_MM) == 33, len(SERVO_MM)
assert sorted(SERVO_MM) == list(range(1, 34))

# ------------------------------------------- CAD 实测坐标覆盖（权威数据源）
CAD_MAPPING_PATH = (pathlib.Path(__file__).resolve().parent /
                    "cad_geometry" / "servo_id_mapping.json")
if CAD_MAPPING_PATH.exists():
    _cad = json.loads(CAD_MAPPING_PATH.read_text(encoding="utf-8"))
    for _key, _rec in _cad["by_id"].items():
        _sid = int(_key)
        _g = SERVO_MM[_sid]
        _g["estimated_xyz_mm"] = tuple(_rec["old_xyz_face_mm"])
        _g["cad_part"] = _rec.get("cad_part")
        _g["cad_owner"] = _rec.get("cad_owner")
        _g["cad_world_mm"] = _rec.get("cad_world_mm")
        _g["match_distance_mm"] = _rec.get("shape_distance_mm")
        _g["confidence"] = _rec["confidence"]
        if _rec.get("cad_xyz_face_mm"):
            # 坐标本身是实测；distance 只反映「通道 ID -> 实体」推断的不确定性
            if (_rec["shape_distance_mm"] or 0) >= 60:
                _g["confidence"] = "cad_measured_low_id_confidence"
            _g["xyz_mm"] = tuple(_rec["cad_xyz_face_mm"])
            _g["vertical_order_source"] = "SolidWorks assembly transform (measured)"
    CAD_MEASURED_COUNT = sum(1 for g in SERVO_MM.values()
                             if g["confidence"].startswith("cad_measured"))
else:
    CAD_MEASURED_COUNT = 0

# 可视化用的模块层：CAD 实测各模块舵机 Z 范围的中点（face mm）
MODULE_PLANES_MM = {"neck": -215.4, "mouth": -76.0,
                    "eye": -15.8, "eyebrow": 48.3}
# 可视化平面的前后深度：CAD 实测舵机 face_y 的中位量级
MODULE_DEPTH_Y_MM = 60.0


# ------------------------------------------------------------------ 配准
def _linear_fit(pairs):
    """一维最小二乘 b = scale*p + offset，返回 (scale, offset, r2, rms)。"""
    n = len(pairs)
    pm = sum(p for p, _ in pairs) / n
    bm = sum(b for _, b in pairs) / n
    sxx = sum((p - pm) ** 2 for p, _ in pairs)
    sxy = sum((p - pm) * (b - bm) for p, b in pairs)
    if sxx < 1e-12:
        return 0.0, bm, 0.0, 0.0
    scale = sxy / sxx
    offset = bm - scale * pm
    sst = sum((b - bm) ** 2 for _, b in pairs)
    sse = sum((b - (scale * p + offset)) ** 2 for p, b in pairs)
    r2 = 1.0 - sse / sst if sst > 1e-12 else 1.0
    return scale, offset, r2, (sse / n) ** 0.5


def fit_anchors(anchors_blender):
    """用面部控制点把物理 mm 坐标系配准到当前 .blend。

    anchors_blender: {name: (x,y,z)}，一般取 Targets 集合各 Empty 的 o.location。
    X / Z 独立最小二乘；Y 用 Z 的 scale 保持深度各向同性、offset 由眼锚点定，
    因为脸颊控制点的 y 在旧模型里并不严格符合真人前后关系。
    """
    pairs = [(ANCHORS_MM[k][i], tuple(anchors_blender[k])[i])
             for k in ANCHORS_MM if k in anchors_blender for i in range(3)]
    used = [k for k in ANCHORS_MM if k in anchors_blender]
    if len(used) < 6:
        raise ValueError("可用锚点不足 6 个，无法配准：%s" % used)

    fit = {"anchors_used": used, "n": len(used), "axis": {}}
    for i, ax in enumerate("XYZ"):
        prs = [(ANCHORS_MM[k][i], tuple(anchors_blender[k])[i]) for k in used]
        sc, off, r2, rms = _linear_fit(prs)
        fit["axis"][ax] = {"scale": sc, "offset": off, "r2": r2, "rms": rms,
                           "method": "least_squares"}

    sx = fit["axis"]["X"]["scale"]
    sz = fit["axis"]["Z"]["scale"]
    if sx <= 0 or sz <= 0:
        raise RuntimeError("配准 scale 非正，检查 .blend 控制点坐标")

    # Y：与 Z 同尺度，平移使眼锚点的物理 y 对齐 Blender y
    scale_y = sz
    eye_y_mm = ANCHORS_MM["eyeE.L"][1]
    eye_y_bu = tuple(anchors_blender["eyeE.L"])[1]
    fit["axis"]["Y"] = {
        "scale": scale_y, "offset": eye_y_bu - scale_y * eye_y_mm,
        "r2": fit["axis"]["Y"]["r2"], "rms": fit["axis"]["Y"]["rms"],
        "method": "isotropic_with_Z + eye_anchor_offset",
    }
    fit["mm_per_blender_unit"] = {
        "X": 1.0 / sx, "Y": 1.0 / scale_y, "Z": 1.0 / sz,
    }
    return fit


def to_blender(xyz_mm, fit):
    """物理 (x,y,z) mm -> .blend 局部坐标。"""
    a = fit["axis"]
    return tuple(round(a[ax]["scale"] * xyz_mm[i] + a[ax]["offset"], 6)
                 for i, ax in enumerate("XYZ"))


def geometry_table(fit=None):
    """返回 33 行的完整几何表（含 Blender 坐标，若给了 fit）。"""
    rows = []
    for sid in range(1, 34):
        g = dict(SERVO_MM[sid])
        g["xyz_mm"] = list(g["xyz_mm"])
        if fit:
            g["xyz_blender"] = list(to_blender(g["xyz_mm"], fit))
        rows.append(g)
    return rows


def read_anchors_from_bpy():
    """在 Blender 内调用：读取 Targets 控制点的局部坐标。"""
    import bpy
    out = {}
    for name in ANCHORS_MM:
        o = bpy.data.objects.get(name)
        if o is not None:
            out[name] = tuple(float(v) for v in o.location)
    return out


if __name__ == "__main__":
    import json
    print("servo_geometry: %d servos" % len(SERVO_MM))
    print("module planes (mm):", MODULE_PLANES_MM)
    for sid in (1, 3, 5, 7, 8, 23, 31):
        g = SERVO_MM[sid]
        print("  M%02d %-8s xyz=%s rank=%2d %s" %
              (sid, g["module"], g["xyz_mm"], g["fig7_rank"], g["note"]))
    print("\nanchors: %d (call fit_anchors(read_anchors_from_bpy()) inside Blender)" %
          len(ANCHORS_MM))
