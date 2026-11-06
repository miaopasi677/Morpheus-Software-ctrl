"""从 servo_map_report.json 生成 33 路舵机真实几何的正视/侧视验证图。"""
import json, pathlib, math
from PIL import Image, ImageDraw, ImageFont

ROOT = pathlib.Path("C:/file/project/Morpheus-Software")
rep = json.loads((ROOT / "servo_map_report.json").read_text(encoding="utf-8"))
rows = rep["servos"]
facts = rep["geometry"]["hardware_facts"]

W, H = 1700, 1000
BG = (248, 249, 251)
INK = (28, 34, 44)
GRID = (222, 226, 232)
COLORS = {
    "tendon": (214, 76, 70),
    "eyebrow": (150, 92, 196),
    "eye": (44, 118, 206),
    "mouth": (32, 148, 116),
    "neck": (196, 132, 34),
}
try:
    F = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 15)
    FB = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 19)
    FS = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 12)
except Exception:
    F = FB = FS = ImageFont.load_default()

im = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(im)
d.text((28, 18), "Morpheus 33-servo MEASURED geometry  |  SolidWorks 2022 full_assembly.SLDASM (32 physical servo instances)", font=FB, fill=INK)
d.text((28, 46), "Physical frame: origin = eye midpoint, +X = subject LEFT, +Y = back of head, +Z = up   |   XYZ from assembly transforms; M31 estimated (no CAD instance)", font=FS, fill=(95, 104, 118))

# 自适应范围：颈舵机实测下探到 -215 mm，旧图的 -145..115 会裁掉
xyz = [r["xyz_mm"] for r in rows if r.get("xyz_mm")]
xs = [p[0] for p in xyz]; ys = [p[1] for p in xyz]; zs = [p[2] for p in xyz]


def _range(vals, step, pad):
    return (math.floor((min(vals) - pad) / step) * step,
            math.ceil((max(vals) + pad) / step) * step)


XR, YR, ZR = _range(xs, 20, 12), _range(ys, 20, 12), _range(zs, 20, 12)
ZSTEP = 50 if (ZR[1] - ZR[0]) > 220 else 20
planes = {m: (a + b) / 2 for m, (a, b) in
          facts["cad_measured"]["servo_face_z_range_mm"].items()}

panels = [
    {"title": "FRONT VIEW  (X vs Z, mm)", "ox": 60, "oy": 90, "w": 780, "h": 760,
     "ax": 0, "az": 2, "xr": XR, "zr": ZR},
    {"title": "SIDE VIEW  (Y vs Z, mm)", "ox": 880, "oy": 90, "w": 780, "h": 760,
     "ax": 1, "az": 2, "xr": YR, "zr": ZR},
]

for p in panels:
    ox, oy, w, h = p["ox"], p["oy"], p["w"], p["h"]
    d.rectangle([ox, oy, ox + w, oy + h], outline=(190, 196, 205), width=2, fill=(255, 255, 255))
    d.text((ox + 12, oy + 10), p["title"], font=FB, fill=INK)
    x0, x1 = p["xr"]
    z0, z1 = p["zr"]

    def uv(x, z):
        return (ox + (x - x0) / (x1 - x0) * w,
                oy + h - (z - z0) / (z1 - z0) * h)

    for gx in range(int(x0), int(x1) + 1, 20):
        if x0 <= gx <= x1:
            a = uv(gx, z0); b = uv(gx, z1)
            d.line([a, b], fill=GRID, width=1)
            d.text((a[0] + 3, oy + h - 22), str(gx), font=FS, fill=(140, 148, 160))
    for gz in range(int(z0), int(z1) + 1, ZSTEP):
        if z0 <= gz <= z1:
            a = uv(x0, gz); b = uv(x1, gz)
            d.line([a, b], fill=GRID, width=1)
            d.text((ox + 5, a[1] - 8), str(gz), font=FS, fill=(140, 148, 160))

    # 模块基准层：CAD 实测各模块舵机 Z 范围中点
    for mod, z in planes.items():
        if z0 <= z <= z1:
            a = uv(x0, z); b = uv(x1, z)
            d.line([a, b], fill=(238, 196, 118), width=2)
            d.text((b[0] - 132, a[1] - 16), "%s  z=%+.0f" % (mod, z), font=FS, fill=(168, 118, 32))

    # 舵机：侧视图里左右对称点会投影重合，合并成 "5/26" 这类标签
    if p["ax"] == 1:
        groups = {}
        for r in rows:
            x, y, z = r["xyz_mm"]
            groups.setdefault((round(y / 6), round(z / 6)), []).append(r)
        items = []
        for grp in groups.values():
            y = sum(g["xyz_mm"][1] for g in grp) / len(grp)
            z = sum(g["xyz_mm"][2] for g in grp) / len(grp)
            items.append((y, z, "/".join(str(g["id"]) for g in grp), grp[0]["module"]))
    else:
        items = [(r["xyz_mm"][0], r["xyz_mm"][2], str(r["id"]), r["module"])
                 for r in rows]

    for px, z, label, module in sorted(items, key=lambda t: (-t[1], t[0])):
        u, v = uv(px, z)
        c = COLORS.get(module, (80, 80, 80))
        r0 = 8 if module == "tendon" else 6
        d.ellipse([u - r0, v - r0, u + r0, v + r0], fill=c, outline=(255, 255, 255), width=1)
        flip = (hash(label) % 2 == 0)
        d.text((u + (-30 if flip else 11), v - (12 if flip else -1)),
               label, font=FS, fill=(35, 40, 50))

# 图例
lx, ly = 60, 878
d.text((lx, ly), "MODULE", font=FB, fill=INK)
for i, (m, c) in enumerate(COLORS.items()):
    x = lx + 95 + i * 145
    d.ellipse([x, ly + 3, x + 14, ly + 17], fill=c)
    cnt = sum(1 for r in rows if r["module"] == m)
    d.text((x + 20, ly + 1), "%s (%d)" % (m, cnt), font=F, fill=INK)
d.text((lx, ly + 32), "Blender: 33 Servo_Rig empties + Morpheus_Servo_Rig (33 bones) + Servo_Links   |   layer marks = CAD measured servo Z midpoints", font=FS, fill=(95, 104, 118))

fit = rep["summary"]["fit_r2"]
d.text((880, 878), "FIT TO .blend", font=FB, fill=INK)
d.text((880, 906), "1 unit = %.1f mm (X) / %.1f mm (Z)   R2: X %.3f  Z %.3f" % (
    rep["summary"]["mm_per_blender_unit"]["X"],
    rep["summary"]["mm_per_blender_unit"]["Z"], fit["X"], fit["Z"]), font=F, fill=INK)
o = rep["summary"]["offset_original_bone_to_servo_axis_units"]
d.text((880, 932), "Old bone head -> real servo axis: mean %.0f mm, max %.0f mm (25 channels)" % (
    o["mean_mm"], o["max"] * rep["summary"]["mm_per_blender_unit"]["Z"]), font=F, fill=(178, 66, 58))
d.text((880, 958), "32/33 axes measured in SolidWorks; ID-to-instance pairing inferred (servo_id_mapping.json)", font=FS, fill=(95, 104, 118))

out = ROOT / "servo_geometry_preview.png"
im.save(out)
print("PREVIEW_OK", out, im.size)
