"""生成映射审计「修复前 → 修复后」对比图。

运行：uv run --with pillow make_mapping_audit_preview.py
数据：mapping_audit.json（Blender 实测，修复后）+ cad_geometry/servo_drive_chains.json
      + cad_geometry/linkage_model.json（机构分型）。
修复前数字来自本会话早期审计运行（transcript 存档），作为常量对照。
"""
import json
import pathlib
from PIL import Image, ImageDraw, ImageFont

ROOT = pathlib.Path(__file__).resolve().parent
audit = json.loads((ROOT / "mapping_audit.json").read_text(encoding="utf-8"))
chains = json.loads((ROOT / "cad_geometry" / "servo_drive_chains.json").read_text(
    encoding="utf-8"))["servos"]
model = json.loads((ROOT / "cad_geometry" / "linkage_model.json").read_text(
    encoding="utf-8"))["servos"]
import servo_map as sm

MOD_COLOR = {
    "tendon": (214, 76, 70),
    "brow": (150, 92, 196),
    "eye": (44, 118, 206),
    "mouth": (32, 148, 116),
    "neck": (196, 132, 34),
}
GRAY = (120, 128, 138)
INK = (28, 34, 44)
RED = (200, 40, 40)
GREEN = (20, 140, 70)
BG = (248, 249, 251)
GRID = (226, 230, 236)

try:
    F = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 15)
    FB = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc", 20)
    FS = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 12)
except Exception:
    F = FB = FS = ImageFont.load_default()


def world_to_face(w):
    return (w[2] - 1.0, w[0] + 96.84, w[1] - 285.87)


points = {}
for r in audit["static"]:
    sid = r["id"]
    ch = chains.get(str(sid)) or {}
    servo = world_to_face(ch["servo_t_mm"]) if ch.get("servo_t_mm") else None
    driver = world_to_face(ch["driver_point_mm"]) if ch.get("driver_point_mm") else None
    points[sid] = {"head": r["head_face_mm"], "servo": servo, "driver": driver,
                   "module": sm.BY_ID[sid]["group"], "name_cn": sm.BY_ID[sid]["name_cn"],
                   "gap": r["dist_head_to_cad_driver_mm"]}

# ---- 实测统计（修复后） ----
ds = [r["dist_head_to_servo_axis_mm"] for r in audit["static"]
      if r["dist_head_to_servo_axis_mm"] is not None]
mean_servo = sum(ds) / len(ds)
skin = {r["id"]: r for r in audit["skin_trajectory"]}
motion = {r["id"]: r for r in audit["motion"]}
eyes = audit.get("eyes_rotation_deg", {})
eye_x = eyes.get("eyes.L", [0])[0]
types = {}
for r in model.values():
    types[r["type"]] = types.get(r["type"], 0) + 1


def ang(sid):
    r = skin.get(sid) or {}
    return r.get("angle_to_pull_dir_deg")


W, H = 1920, 1080
im = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(im)
d.text((24, 14), "映射审计（修复后）：33 骨 = CAD 真实机构轨迹 · 无 IK / 无求解器 / 无 LUT", font=FB, fill=INK)
d.text((24, 46), "head 落在真实铰链 / 表皮锚点（平均距舵机轴心 %.1fmm）；ARKit 归一化值同源驱动骨骼与 PWM，PWM 下发路径零改动" % mean_servo, font=F, fill=GRAY)

# ------------------------------------------------ 视图面板
def draw_panel(box, title, hx, hz, xlim, zlim):
    x0, y0, x1, y1 = box
    d.rectangle(box, outline=(200, 205, 212), width=1, fill=(255, 255, 255))
    d.text((x0 + 12, y0 + 8), title, font=FB, fill=INK)

    def px(fx, fz):
        return (x0 + 40 + (fx - xlim[0]) / (xlim[1] - xlim[0]) * (x1 - x0 - 80),
                y1 - 40 - (fz - zlim[0]) / (zlim[1] - zlim[0]) * (y1 - y0 - 80))

    # 网格
    gx = xlim[0]
    while gx <= xlim[1]:
        p = px(gx, 0)
        d.line([(p[0], y0 + 40), (p[0], y1 - 40)], fill=GRID, width=1)
        d.text((p[0] - 12, y1 - 34), str(int(gx)), font=FS, fill=GRAY)
        gx += 30
    gz = zlim[0]
    while gz <= zlim[1]:
        p = px(0, gz)
        d.line([(x0 + 40, p[1]), (x1 - 40, p[1])], fill=GRID, width=1)
        d.text((x0 + 6, p[1] - 8), str(int(gz)), font=FS, fill=GRAY)
        gz += 30

    for sid, p in points.items():
        c = MOD_COLOR[p["module"]]
        h = px(p["head"][hx], p["head"][hz])
        if p["servo"]:
            s = px(p["servo"][hx], p["servo"][hz])
            d.line([h, s], fill=(190, 196, 205), width=1)
            d.rectangle([s[0] - 4, s[1] - 4, s[0] + 4, s[1] + 4], outline=GRAY, width=2)
        if p["driver"]:
            t = px(p["driver"][hx], p["driver"][hz])
            if p["gap"] is not None and p["gap"] > 30:
                d.line([h, t], fill=RED, width=2)
            d.polygon([(t[0], t[1] - 6), (t[0] - 6, t[1] + 5), (t[0] + 6, t[1] + 5)],
                      outline=GREEN, width=2)
        d.ellipse([h[0] - 5, h[1] - 5, h[0] + 5, h[1] + 5], fill=c)
        d.text((h[0] + 7, h[1] - 8), "M%02d" % sid, font=FS, fill=c)
    return px


draw_panel((24, 90, 640, 900), "正视（face X-Z，mm）", 0, 2, (-95, 95), (-230, 80))
draw_panel((664, 90, 1280, 900), "侧视（face Y-Z，mm；+Y=脑后）", 1, 2, (-120, 100), (-230, 80))

# ------------------------------------------------ 对比面板
x0, y0 = 1304, 90
d.rectangle([x0, y0, W - 24, 900], outline=(200, 205, 212), fill=(255, 255, 255))
d.text((x0 + 14, y0 + 8), "修复前 → 修复后（注入 ARKit=1 实测）", font=FB, fill=INK)

cx = (x0 + 16, x0 + 270, x0 + 410)
yy = y0 + 46
d.text((cx[0], yy), "通道 · 指标", font=F, fill=GRAY)
d.text((cx[1], yy), "修复前", font=F, fill=RED)
d.text((cx[2], yy), "修复后", font=F, fill=GREEN)
yy += 28
rows = [
    ("M01 颊 · 拉动偏角", "161.2°", "%.1f°" % ang(1)),
    ("M02 鼻 · 拉动偏角", "164.0°", "%.1f°" % ang(2)),
    ("M05 眼睑 · 拉动偏角", "168.0°", "%.1f°" % ang(5)),
    ("M04 眉峰 · 输出转角", "63.8°*", "%.1f°" % motion[4]["rot_deg"]),
    ("M09 嘴角 · 摇杆转角", "0°", "%.1f°" % motion[9]["rot_deg"]),
    ("M23 下颌 · 运动", "原地 2.4°", "后拉张口"),
    ("M16 下唇 · 轨轴", "无运动", "竖直向下"),
    ("眼球 · 抬头注视", "+43.2° ↓", "%.1f° ↑" % eye_x),
    ("蒙皮 · 顶点组", "14 路缺失", "31 组"),
    ("M31 颈 · 顶点垄断", "10348", "修剪 9529"),
]
for label, before, after in rows:
    d.text((cx[0], yy), label, font=F, fill=INK)
    d.text((cx[1], yy), before, font=F, fill=RED)
    d.text((cx[2], yy), after, font=F, fill=GREEN)
    yy += 27

yy += 10
d.line([(x0 + 14, yy), (W - 38, yy)], fill=GRID, width=1)
yy += 12
notes = [
    "* M04 修复前含 111.4° 中立跳支；修复后为闭式圆-圆交点真值。",
    "机构分型：线驱%d(含下颌) · 四连杆%d · 推杆%d · 齿条%d · 眼球%d · 颈%d · 一阶%d" % (
        types.get("cable", 0), types.get("linkage", 0), types.get("pushrod", 0),
        types.get("rack", 0), types.get("eye_rev", 0), types.get("neck_rev", 0),
        types.get("open", 0)),
    "正向：曲柄角→摇杆角闭式求解；线驱/齿条/推杆沿真实方向平动。",
    "同源：ARKit→frame_to_pwm→33 PWM；骨转角=(PWM−start)×π/2000。",
    "PWM 下发（servo_map / emit_pwm）零改动。",
    "遗留：11 路一阶近似 + anchor_proxy 挂点待实物标定。",
]
for i, txt in enumerate(notes):
    d.text((x0 + 14, yy), txt, font=FS if i in (0, 1) else F,
           fill=GRAY if i in (0, 5) else INK)
    yy += 24 if i in (0, 1) else 28

# ------------------------------------------------ 图例
ly = 930
d.text((24, ly), "图例：", font=FB, fill=INK)
items = [("● 骨骼 head（真实铰链/锚点）", MOD_COLOR["mouth"]),
         ("□ 舵机轴心（CAD）", GRAY),
         ("△ CAD 传动链末端", GREEN),
         ("— head≠末端 >30mm（固定铰/球心类）", RED),
         ("灰线 = head→舵机轴心参考", (190, 196, 205))]
xx = 110
for txt, c in items:
    d.text((xx, ly + 3), txt, font=F, fill=c)
    xx += d.textlength(txt, font=F) + 30
d.text((24, ly + 36), "模块颜色：", font=F, fill=INK)
xx = 120
for mod, label in [("tendon", "线驱"), ("brow", "眉"), ("eye", "眼"), ("mouth", "嘴"), ("neck", "颈")]:
    d.ellipse([xx, ly + 40, xx + 12, ly + 52], fill=MOD_COLOR[mod])
    d.text((xx + 18, ly + 36), label, font=F, fill=INK)
    xx += 90
d.text((24, ly + 70), "数据：mapping_audit.json（Blender 5.2.3 后台实测 LY_GENERAL_V0.blend，2026-11-09）＋ cad_geometry/{servo_drive_chains,linkage_model}.json；修复前数字来自本会话早期审计存档", font=FS, fill=GRAY)

out = ROOT / "mapping_audit_preview.png"
im.save(out)
print("saved", out)