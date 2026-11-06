"""
Morpheus 33 舵机 <-> LY_GENERAL_V0.blend 骨骼 / 控制点 映射表（单一事实来源）。

数据来源：
  - arXiv:2507.16645v1 附录 Fig.7  : 舵机 ID 1..30 的名称与控制区域
  - arXiv:2507.16645v1 附录 Table V: 每个舵机的 Control Position Lower Bound /
                                    Start Value / Upper Bound（原始 PWM，0..2000）
  - 硬件仓库 Morpheus-Hardware README BOM: 29x GUOHUA A0090 9g + 4x KS3518/TD-8035MG

约定：
  - lower / upper 是「控制命令允许区间的两端」，不是数值大小关系。
    部分通道 upper < lower（例如 ID=1: 1800 -> 1400），代表舵机安装方向相反。
    因此 pwm_low=min(lower,upper)，pwm_high=max(lower,upper)，direction 记录方向。
  - start 是机械中立位，所有未参与逆解的通道保持 start。
  - inverse=True 表示该通道参与 Face Inverse Learning 的 26 维优化。
    论文正文排除的是 7/24(眼球注视 2)、11/12(舌 2)、31/32/33(颈 3)，
    共 33-7=26 路。附录写的 "1-6, 8-10, 13-23, 24-30" 合计 27 且含 24，
    与正文矛盾，此处按正文的 26 路处理（即 24-30 应为 25-30）。
  - 旧模型 .l 在 Blender +X 侧（人物自身左），对应论文的 "Left"；.r 对应 "Right"。
"""

PAPER = "arXiv:2507.16645v1 Fig.7 / Table V"

# id, 英文名称, 中文说明, 模块, lower, start, upper, 逆解通道, 控制点 EMPTY, 对应骨骼
_RAW = [
    # bones 只列【直接用于原 get_position.py 角度输出的骨骼】。
    # 不再混入 Brow_In/Lip_Up 等定位骨骼，也不为缺失通道创建假骨骼。
    # 7/24 在新硬件上左右联动，合并原左右骨骼；yaw 两侧旧硬件方向相反，权重见 SOURCE_WEIGHTS。
    (1,  "Left Cheek",                    "左颊 线驱",        "tendon", 1800, 1800, 1400, True,  "cheekE.L",       []),
    (2,  "Left Nose",                     "左鼻 线驱",        "tendon", 800,  800,  0,    True,  "noseE.L",        []),
    (3,  "Left Eyebrow Center",           "左眉头",           "brow",   980,  1080, 1480, True,  "browInnerE.L",   ["Brow_In_Rotation_pitch.l"]),
    (4,  "Left Eyebrow Peak",             "左眉峰",           "brow",   1100, 800,  500,  True,  "browOuterE.L",   ["Brow_Out_Rotation.l"]),
    (5,  "Left Upper Eyelid",             "左上眼睑",         "eye",    1700, 1300, 600,  True,  "eyeLidUpE.L",    ["Eyelid_Up.l"]),
    (6,  "Left Lower Eyelid",             "左下眼睑",         "eye",    1000, 1200, 1800, True,  "eyeLidDown.L",   ["Eyelid_Dwn.l"]),
    (7,  "Eyes Up/Down Gaze",             "眼球俯仰(联动)",    "eye",    300,  1000, 1500, False, "eyeE.L",         ["Eye_pitch.r", "Eye_pitch.l"]),
    (8,  "Upper Lip Center",              "上唇中央",         "mouth",  1000, 1000, 600,  True,  "lipUpE.C",       ["Lip_Up_Rotation_C"]),
    (9,  "Left Upper Corner Mouth",       "左嘴角上",         "mouth",  500,  1000, 1500, True,  "lipConnerE.L",   ["Smile_Rotation.l"]),
    (10, "Upper Lip Right",               "上唇右",           "mouth",  1000, 1000, 1200, True,  "lipUpE.R",       ["Lip_Up_Rotation.r"]),
    (11, "Tongue Extension/Retraction",   "舌伸缩",           "mouth",  800,  1000, 1200, False, None,             []),
    (12, "Tongue Up/Down",                "舌上下",           "mouth",  800,  500,  300,  False, None,             []),
    (13, "Chin (passive)",                "下巴 被动",        "mouth",  1200, 1600, 1600, True,  None,             []),
    (14, "Left Lower Corner Mouth",       "左嘴角下",         "mouth",  1000, 1000, 1500, True,  "lipConnerE.L",   ["Lip_Corner_Rotation.l"]),
    (15, "Lower Lip Center",              "下唇中央",         "mouth",  1300, 1300, 1900, True,  "lipDownE.C",     ["Lip_Dwn_Rotation_C"]),
    (16, "Lower Lip Left",                "下唇左",           "mouth",  1500, 1500, 800,  True,  "lipDownE.L",     ["Lip_Dwn_Rotation.l"]),
    (17, "Lower Lip Right",               "下唇右",           "mouth",  800,  800,  1200, True,  "lipDownE.R",     ["Lip_Dwn_Rotation.r"]),
    (18, "Chin (active)",                 "下巴 主动",        "mouth",  1200, 1600, 1600, True,  None,             []),
    (19, "Right Lower Corner Mouth",      "右嘴角下",         "mouth",  1000, 1000, 500,  True,  "lipConnerE.R",   ["Lip_Corner_Rotation.r"]),
    (20, "Jaw Left/Right",                "下颌左右",         "mouth",  400,  1000, 1600, True,  "jawE",           ["Jaw_Yaw"]),
    (21, "Upper Lip Left",                "上唇左",           "mouth",  1000, 1000, 800,  True,  "lipUpE.L",       ["Lip_Up_Rotation.l"]),
    (22, "Right Upper Corner Mouth",      "右嘴角上",         "mouth",  1500, 1000, 500,  True,  "lipConnerE.R",   ["Smile_Rotation.r"]),
    (23, "Mouth Opening/Closing",         "张合下颌",         "mouth",  1150, 1050, 1050, True,  "jawE",           ["Jaw_Open"]),
    (24, "Eyes Left/Right Gaze",          "眼球左右(联动)",    "eye",    800,  1150, 1500, False, "eyeE.R",         ["Eye_yaw.r", "Eye_yaw.l"]),
    (25, "Right Lower Eyelid",            "右下眼睑",         "eye",    1200, 1000, 400,  True,  "eyeLidDown.R",   ["Eyelid_Dwn.r"]),
    (26, "Right Upper Eyelid",            "右上眼睑",         "eye",    500,  1000, 1600, True,  "eyeLidUpE.R",    ["Eyelid_Up.r"]),
    (27, "Right Eyebrow Peak",            "右眉峰",           "brow",   900,  1200, 1500, True,  "browOuterE.R",   ["Brow_Out_Rotation.r"]),
    (28, "Right Eyebrow Center",          "右眉头",           "brow",   1300, 1200, 800,  True,  "browInnerE.R",   ["Brow_In_Rotation_pitch.r"]),
    (29, "Right Nose",                    "右鼻 线驱",        "tendon", 1200, 1200, 2000, True,  "noseE.R",        []),
    (30, "Right Cheek",                   "右颊 线驱",        "tendon", 0,    0,    600,  True,  "cheekE.R",       []),
    (31, "Neck Rotation",                 "颈 旋转",          "neck",   1500, 1750, 2000, False, None,             ["Neck_1"]),
    (32, "Neck Nodding/Shaking (left)",   "颈 点头/摇头 左",   "neck",   900,  1200, 1300, False, None,             ["Neck_2"]),
    (33, "Neck Nodding/Shaking (right)",  "颈 点头/摇头 右",   "neck",   1200, 900,  800,  False, None,             ["Neck_3"]),
]

SERVOS = []
for (sid, name, name_cn, group, lower, start, upper, inverse, target, bones) in _RAW:
    SERVOS.append({
        "id": sid,
        "name": name,
        "name_cn": name_cn,
        "group": group,
        "lower": lower,
        "start": start,
        "upper": upper,
        "pwm_low": min(lower, upper),
        "pwm_high": max(lower, upper),
        "pwm_start": start,
        "direction": -1 if upper < lower else 1,
        "inverse": inverse,
        "target": target,
        "bones": bones,
    })

BY_ID = {s["id"]: s for s in SERVOS}
INVERSE_IDS = [s["id"] for s in SERVOS if s["inverse"]]
assert len(SERVOS) == 33
assert len(INVERSE_IDS) == 26

# 新硬件的双眼各只有一路联动舵机。pitch 两侧原标定同向，直接平均；
# yaw 两侧旧硬件 reverse_direction 相反，因此左源取负后再平均，避免同向注视被抵消。
SOURCE_WEIGHTS = {
    7: {"Eye_pitch.r": 0.5, "Eye_pitch.l": 0.5},
    24: {"Eye_yaw.r": 0.5, "Eye_yaw.l": -0.5},
}

# 旧模型中存在、但 Morpheus 硬件没有独立舵机的骨骼（保留，不参与下发）
UNMAPPED_BONES = [
    "Brow_In_Rotation_yaw.l", "Brow_In_Rotation_yaw.r",
    "Lip_Up_Out_C", "Lip_Dwn_Out_C",
    "Face_Root", "Face_Neck", "Skull",
]

# render.py 的 52 维 ARKit 输出顺序（本项目推理结果的列序，务必与 render.py 一致）
MODEL_BS_LIST = [
    "browDownLeft", "browDownRight", "browInnerUp", "browOuterUpLeft", "browOuterUpRight",
    "cheekPuff", "cheekSquintLeft", "cheekSquintRight", "eyeBlinkLeft", "eyeBlinkRight",
    "eyeLookDownLeft", "eyeLookDownRight", "eyeLookInLeft", "eyeLookInRight",
    "eyeLookOutLeft", "eyeLookOutRight", "eyeLookUpLeft", "eyeLookUpRight",
    "eyeSquintLeft", "eyeSquintRight", "eyeWideLeft", "eyeWideRight",
    "jawForward", "jawLeft", "jawOpen", "jawRight", "mouthClose", "mouthDimpleLeft",
    "mouthDimpleRight", "mouthFrownLeft", "mouthFrownRight", "mouthFunnel", "mouthLeft",
    "mouthLowerDownLeft", "mouthLowerDownRight", "mouthPressLeft", "mouthPressRight",
    "mouthPucker", "mouthRight", "mouthRollLower", "mouthRollUpper", "mouthShrugLower",
    "mouthShrugUpper", "mouthSmileLeft", "mouthSmileRight", "mouthStretchLeft",
    "mouthStretchRight", "mouthUpperUpLeft", "mouthUpperUpRight", "noseSneerLeft",
    "noseSneerRight", "tongueOut",
]
assert len(MODEL_BS_LIST) == 52

# 52 维 ARKit -> 26 维逆解通道的「种子」映射（按部位分组，权重为初始猜测）。
# 这不是论文训练出来的 W：论文用 self-modeling MLP + 梯度反传求 W，本表只是
# 让链路能先跑起来的手工初值，必须经 5000 组实测数据重新标定。
ARKIT_TO_SERVO = {
    "browDownLeft":       {3: -0.8, 4: -0.4},
    "browDownRight":      {28: -0.8, 27: -0.4},
    "browInnerUp":        {3: 0.8, 28: 0.8},
    "browOuterUpLeft":    {4: 1.0},
    "browOuterUpRight":   {27: 1.0},
    "eyeLookUpLeft":      {7: 0.8},   # 7/24 由额外 face tracking 控制，此处仅给初值
    "eyeLookUpRight":     {7: 0.8},
    "eyeLookDownLeft":    {7: -0.8},
    "eyeLookDownRight":   {7: -0.8},
    "eyeLookInLeft":      {24: 0.8},
    "eyeLookInRight":     {24: -0.8},
    "eyeLookOutLeft":     {24: -0.8},
    "eyeLookOutRight":    {24: 0.8},
    "cheekPuff":          {1: 0.5, 30: 0.5},
    "cheekSquintLeft":    {1: 0.4, 6: 0.3},
    "cheekSquintRight":   {30: 0.4, 25: 0.3},
    "eyeBlinkLeft":       {5: 1.0, 6: 0.3},
    "eyeBlinkRight":      {26: 1.0, 25: 0.3},
    "eyeSquintLeft":      {6: 0.7},
    "eyeSquintRight":     {25: 0.7},
    "eyeWideLeft":        {5: -0.7},
    "eyeWideRight":       {26: -0.7},
    # ID23 的 Start=Upper=1050，只能向 Lower=1150 方向动，故张嘴系数取负。
    # 这类符号是「种子猜测」，必须按实物逐通道扫一遍确认。
    "jawOpen":            {23: -1.0},
    "jawLeft":            {20: -0.7},
    "jawRight":           {20: 0.7},
    "jawForward":         {20: 0.2},
    "mouthSmileLeft":     {9: 1.0, 14: 0.6},
    "mouthSmileRight":    {22: 1.0, 19: 0.6},
    "mouthFrownLeft":     {14: -0.8},
    "mouthFrownRight":    {19: -0.8},
    "mouthLowerDownLeft": {16: 0.9},
    "mouthLowerDownRight":{17: 0.9},
    "mouthUpperUpLeft":   {21: 0.9},
    "mouthUpperUpRight":  {10: 0.9},
    "mouthPucker":        {8: 0.6, 15: 0.6, 21: -0.3, 10: -0.3},
    "mouthFunnel":        {8: 0.5, 15: 0.5},
    "mouthLeft":          {9: 0.4, 22: -0.4, 14: 0.4, 19: -0.4},
    "mouthRight":         {9: -0.4, 22: 0.4, 14: -0.4, 19: 0.4},
    "mouthStretchLeft":   {9: 0.5, 14: 0.5},
    "mouthStretchRight":  {22: 0.5, 19: 0.5},
    "mouthDimpleLeft":    {9: 0.3},
    "mouthDimpleRight":   {22: 0.3},
    "mouthPressLeft":     {21: 0.3, 16: 0.3},
    "mouthPressRight":    {10: 0.3, 17: 0.3},
    "mouthClose":         {23: 0.3},
    "mouthRollLower":     {15: 0.3},
    "mouthRollUpper":     {8: 0.3},
    "mouthShrugLower":    {15: 0.3},
    "mouthShrugUpper":    {8: 0.3},
    "noseSneerLeft":      {2: 0.9},
    "noseSneerRight":     {29: 0.9},
    "tongueOut":          {11: 0.9},
}


def bone_to_servo(bone_name):
    """返回该骨骼对应的舵机 dict；未映射返回 None。"""
    for s in SERVOS:
        if bone_name in s["bones"]:
            return s
    return None


def target_to_servos(target_name):
    """一个控制点可能服务多个舵机（如嘴角 9/14），返回列表。"""
    return [s for s in SERVOS if s["target"] == target_name]


def norm_to_pwm(servo_id, value):
    """归一化 [0,1] -> 该舵机 PWM 命令，自动按方向反解并 clamp 到允许区间。"""
    s = BY_ID[servo_id]
    v = max(0.0, min(1.0, float(value)))
    if s["direction"] < 0:
        v = 1.0 - v
    return int(round(s["pwm_low"] + v * (s["pwm_high"] - s["pwm_low"])))


def frame_to_pwm(arkit_vector):
    """52 维 ARKit 系数 -> 33 维 PWM 命令（未参与通道保持 Start Value）。"""
    out = [s["pwm_start"] for s in SERVOS]
    accum = {}
    for i, name in enumerate(MODEL_BS_LIST):
        w = float(arkit_vector[i])
        for sid, coeff in ARKIT_TO_SERVO.get(name, {}).items():
            accum[sid] = accum.get(sid, 0.0) + w * coeff
    for sid, raw in accum.items():
        s = BY_ID[sid]
        # 直接在物理 PWM 空间做：以 Start Value 对应的位置为基准，
        # 按舵机安装方向叠加表情偏移（direction<0 表示装反，正向表情 = PWM 减小）。
        # 不要再套 norm_to_pwm()，否则方向会被翻转两次。
        span = max(1, s["pwm_high"] - s["pwm_low"])
        base = (s["pwm_start"] - s["pwm_low"]) / span
        delta = 0.5 * raw * (1 if s["direction"] > 0 else -1)
        v = max(0.0, min(1.0, base + delta))
        out[sid - 1] = int(round(s["pwm_low"] + v * span))
    return out


if __name__ == "__main__":
    print("servo_map: %d servos, %d inverse channels" % (len(SERVOS), len(INVERSE_IDS)))
    print("inverse ids:", INVERSE_IDS)
    for s in SERVOS:
        print("%2d %-34s low=%4d start=%4d high=%4d dir=%+d target=%-16s bones=%s" %
              (s["id"], s["name"], s["pwm_low"], s["pwm_start"], s["pwm_high"],
               s["direction"], s["target"] or "-", ",".join(s["bones"]) or "MISSING"))
