"""从 SolidWorks 装配几何追踪每一路舵机的传动链，并定位表皮驱动点。

思路
----
1. 读取 cad_geometry/assembly_instances.json 的 280 个实例，按 row-vector 约定
   (world = local @ R + t) 把对应 STL 变换到世界坐标。
2. 用 AABB 预筛 + 点云最近点，建立零件之间的真实接触图（间隙阈值可调）。
3. 以每个舵机实例为起点做 BFS，只沿“传动件”扩展：
   曲柄 crank / 连杆 rod / 钢丝 wire / 球铰 ball / 球头 OT / 滑轨 slider /
   齿条 rack / 齿轮 gear / 五连杆·六连杆 mechanism / 摆杆 pendulum / 摇杆 rocker /
   摇臂 joystick / 连接件 connector / 拉杆柄 shank / 手柄 handle …
   安装板、底板、轴承盖等结构件不作为通路。
4. 记录传动链接触到的“表皮/被驱动件”：shell_skin_face、眼睑 eyelid、
   唇 lip、牙 teeth、颊 cheek、上唇连接点 connection_point、下颌 jaw。
5. 驱动点取传动件与表皮件最近点对中位于表皮上的那一点；若该路没有直接接触表皮，
   退化为链上离舵机最远的点，并标注置信度。

输出 cad_geometry/servo_drive_chains.json

运行（Blender 自带 numpy）：
  blender --background --python trace_drive_chains.py
"""

import json
import math
import pathlib
import struct
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
GEO = ROOT / "cad_geometry"
STL = GEO / "stl"

# ---------------------------------------------------------------- 零件分类
SERVO_RE = ("mg90", "a0090", "ks3518", "guohua")

DRIVE_KEYS = (
    "crank", "rod", "link", "wire", "ball", "ot head", "ot_mirror", "slider",
    "rack", "gear", "mechanism", "pendulum", "rocker", "joystick", "connector",
    "shank", "handle", "hinge", "crossbar", "extension", "regulator", "comb",
    "connector for ot",
)
STRUCT_KEYS = (
    "mounting platform", "mounting bracket", "baseplate", "bearing", "bracket",
    "plate", "installation platform", "board", "bassinet", "mount",
    "stubs", "flange", "camera", "pupil", "raspberry", "nut", "screw",
    "shell_", "carapace", "ears",
)
SKIN_KEYS = (
    "skin_face", "eyelid", "lip", "teeth", "cheek", "connection_point", "jaw",
    "facial_bones",
)


DRIVE_FIRST = ("crank", "rod", "wire", "line", "link", "ball", "ot head",
               "connector", "shank", "handle", "rocker", "joystick", "pendulum",
               "regulator", "comb", "rack", "gear", "slider", "mechanism",
               "crossbar", "extension", "hinge", "arm")


def classify(name):
    low = name.lower()
    if any(k in low for k in SERVO_RE):
        return "servo"
    if "connection_point" in low:
        return "skin"                      # 显式的表皮驱动点标记件
    if "eyeball" in low or "pupil" in low:
        return "skin"                      # 眼球是被驱动对象
    if any(k in low for k in STRUCT_KEYS):
        return "struct"                    # 安装柱/底板/导轨优先判为结构
    if any(k in low for k in DRIVE_FIRST):
        return "drive"
    if any(k in low for k in SKIN_KEYS):
        return "skin"
    return "other"


# ---------------------------------------------------------------- STL
def read_binary_stl(path):
    data = pathlib.Path(path).read_bytes()
    if len(data) < 84:
        return None
    n = struct.unpack("<I", data[80:84])[0]
    if 84 + n * 50 > len(data) or n <= 0:
        n = max(0, (len(data) - 84) // 50)
    if n <= 0:
        return None
    raw = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    pts = tri.astype(np.float64).reshape(-1, 3)
    # 去重后等距降采样，保留形状骨架
    pts = np.unique(np.round(pts, 4), axis=0)
    if len(pts) > 2600:
        idx = np.linspace(0, len(pts) - 1, 2600).astype(int)
        pts = pts[idx]
    return pts


def _norm(name):
    return name.lower().replace(" ", "_").replace("-", "_")


def stl_path_for(item):
    base = item["file"].rsplit(".", 1)[0]
    owner = item["owner_model"]
    norm = _norm(base)
    owner_norm = _norm(owner)
    for c in (STL / (owner + "__" + base + ".stl"),
              STL / (owner + "__" + base.replace(" ", "_") + ".stl"),
              STL / (base.replace(" ", "_") + ".stl"),
              STL / (base + ".stl")):
        if c.exists():
            return c
    for p in STL.glob("*.stl"):
        stem = p.stem
        tail = stem.split("__")[-1]
        if _norm(tail) == norm:
            return p
        if stem.split("__")[0].lower() == owner_norm and _norm(tail) == norm:
            return p
    return None


def load_world_points():
    inst = json.loads((GEO / "assembly_instances.json").read_text(encoding="utf-8"))["instances"]
    cache = {}
    out = []
    for it in inst:
        p = stl_path_for(it)
        pts = None
        if p is not None:
            if p not in cache:
                cache[p] = read_binary_stl(p)
            local = cache[p]
            if local is not None and len(local):
                R = np.array(it["world_R"], dtype=np.float64)
                t = np.array(it["world_t_mm"], dtype=np.float64)
                pts = local @ R + t          # row-vector 约定
        rec = dict(it)
        rec["klass"] = classify(it["file"])
        rec["pts"] = pts
        rec["bbox"] = np.array([it["world_bbox"]["min_mm"],
                                it["world_bbox"]["max_mm"]], dtype=np.float64)
        out.append(rec)
    return out


# ---------------------------------------------------------------- 几何
def bbox_gap(a, b):
    d = np.maximum(0.0, np.maximum(a[0] - b[1], b[0] - a[1]))
    return float(np.linalg.norm(d))


def min_dist(pa, pb):
    """点云最小距离与最近点对，分块广播避免爆内存。"""
    if pa is None or pb is None or not len(pa) or not len(pb):
        return math.inf, None, None
    best = (math.inf, None, None)
    step = 400
    for i in range(0, len(pa), step):
        a = pa[i:i + step]
        d2 = ((a[:, None, :] - pb[None, :, :]) ** 2).sum(-1)
        k = int(d2.argmin())
        val = math.sqrt(float(d2.flat[k]))
        if val < best[0]:
            ia, ib = divmod(k, d2.shape[1])
            best = (val, a[ia], pb[ib])
    return best


def nearest_on(points, ref):
    """points(单点) 到 ref 点云最近点。"""
    d2 = ((ref - points[None, :]) ** 2).sum(-1)
    k = int(d2.argmin())
    return ref[k], math.sqrt(float(d2[k]))


# ---------------------------------------------------------------- 追踪
def trace(servos, items):
    n = len(items)
    # 候选对：bbox 间隙 < 6mm
    cand = {}
    for i in range(n):
        for j in range(i + 1, n):
            g = bbox_gap(items[i]["bbox"], items[j]["bbox"])
            if g <= 6.0:
                cand[(i, j)] = g

    adj = {i: [] for i in range(n)}
    pair_info = {}
    for (i, j), g in cand.items():
        a, b = items[i], items[j]
        if a["klass"] == "struct" and b["klass"] == "struct":
            continue
        d, pa, pb = min_dist(a["pts"], b["pts"])
        # 舵机与传动件是轴孔插配，表面点云到轴心会有几毫米；其余按真实接触。
        limit = 3.0
        if d <= limit and not (a["klass"] == "skin" and b["klass"] == "skin"):
            adj[i].append(j)
            adj[j].append(i)
            pair_info[(i, j)] = (d, pa, pb)
            pair_info[(j, i)] = (d, pb, pa)
    return adj, pair_info


def follow(start, items, adj, pair_info):
    """从舵机出发沿传动方向单链前进。

    规则：
      - 第一跳锁定与舵机输出轴最近、且优先为曲柄/摇杆的传动件（同轴插配）。
      - 之后每跳要求“伸展半径”单调增大，避免在密集子装配里打转。
      - 触到表皮件即记录接触点并停止，不跨子装配扩散。
    """
    owner = items[start]["owner_model"]
    servo_pt = np.array(items[start]["world_t_mm"], dtype=np.float64)

    def reach(it):
        if it["pts"] is None:
            return 0.0
        return float(np.linalg.norm(it["pts"] - servo_pt[None, :], axis=1).max())

    def approach(it):
        if it["pts"] is None:
            return math.inf
        return float(np.linalg.norm(it["pts"] - servo_pt[None, :], axis=1).min())

    CRANKISH = ("crank", "handle", "rocker", "joystick", "pendulum", "arm")

    def is_crank(it):
        low = it["file"].lower()
        return 0 if any(k in low for k in CRANKISH) else 1

    visited = {start}
    chain = []
    skin_hits = []
    cur = start
    prev_reach = 0.0
    for depth in range(14):
        skin_best = None
        drive_best = None
        for nb in adj[cur]:
            if nb in visited:
                continue
            it = items[nb]
            if it["klass"] in ("servo", "struct", "other"):
                continue
            if it["owner_model"] != owner:
                # 只允许柔性线跨子装配（眉/鼻/颊线驱的钢丝挂在 full_assembly 下），
                # 否则贪心链会从眼部串到紧邻的眉部摇杆。
                low_f = it["file"].lower()
                if it["klass"] == "skin":
                    pass
                elif it["klass"] == "drive" and ("wire" in low_f or "line" in low_f):
                    pass
                else:
                    continue
            d, pc, pn = pair_info.get((cur, nb), (math.inf, None, None))
            # 图里已按“点云接触 或 bbox 相交”建边，这里不再二次收紧。
            if it["klass"] == "skin":
                if depth == 0:
                    continue          # 舵机不会直接顶在皮肤上，必须先过传动件
                if pn is not None:
                    score = reach(it)
                    if skin_best is None or score > skin_best[0]:
                        skin_best = (score, nb, pn, d)
                continue
            if depth == 0:
                # 舵机输出轴插在曲柄孔里，两者表面点云几乎重合（实测 ~0.01mm）
                key = is_crank(it) * 100.0 + min(d, 99.0)
                if drive_best is None or key < drive_best[0]:
                    drive_best = (key, nb, pc, d)
            else:
                r = reach(it)
                if r <= prev_reach + 0.5:
                    continue
                if drive_best is None or r > drive_best[0]:
                    drive_best = (r, nb, pc, d)
        if depth > 0 and skin_best is not None:
            skin_hits.append((skin_best[1], skin_best[2], skin_best[3]))
            break
        if drive_best is None:
            if skin_best is not None:
                skin_hits.append((skin_best[1], skin_best[2], skin_best[3]))
            break
        visited.add(drive_best[1])
        chain.append((drive_best[1], drive_best[3]))
        prev_reach = reach(items[drive_best[1]])
        cur = drive_best[1]
    return chain, skin_hits



def main():
    items = load_world_points()
    servos = [i for i, it in enumerate(items) if it["klass"] == "servo"]
    print("INSTANCES", len(items), "SERVOS", len(servos),
          "WITH_PTS", sum(1 for it in items if it["pts"] is not None))
    adj, pair_info = trace(servos, items)
    print("EDGES", sum(len(v) for v in adj.values()) // 2)

    # 表皮参考点云（仅前半脸：CAD x < -90 是脸的前表面）
    skin_all = [it["pts"] for it in items
                if it["file"].lower().startswith("shell_skin_face") and it["pts"] is not None]
    skin_all = np.vstack(skin_all) if skin_all else None
    skin_ref = skin_all[skin_all[:, 0] < -90.0] if skin_all is not None else None
    print("SKIN_POINTS", 0 if skin_ref is None else len(skin_ref))

    by_ref = {it["ref_id"]: i for i, it in enumerate(items)}
    mapping = json.loads((GEO / "servo_id_mapping.json").read_text(encoding="utf-8"))["by_id"]

    def front(pts):
        if pts is None:
            return None
        sel = pts[pts[:, 0] < -90.0]
        return sel if len(sel) else None

    def best_on(pts):
        """零件上最贴近前半脸皮肤的点 -> (点, 距离, 皮肤索引)。"""
        d2 = ((pts[:, None, :] - skin_ref[None, :, :]) ** 2).sum(-1)
        i, j = divmod(int(d2.argmin()), d2.shape[1])
        return pts[i], math.sqrt(float(d2[i, j])), j

    out = {}
    for sid in range(1, 34):
        rec = mapping.get(str(sid)) or {}
        ref = rec.get("cad_ref_id")
        idx = by_ref.get(ref) if ref else None
        if idx is None or items[idx]["klass"] != "servo":
            out[str(sid)] = {"ref": ref, "status": "no_cad_servo",
                             "chain": [], "driver_point_mm": None}
            continue
        chain, hits = follow(idx, items, adj, pair_info)
        servo_pt = np.array(items[idx]["world_t_mm"], dtype=np.float64)

        rows = [{"ref": items[nb]["ref_id"], "file": items[nb]["file"],
                 "owner": items[nb]["owner_model"], "gap_mm": round(gap, 3)}
                for nb, gap in chain]
        hit_rows = [{"ref": items[nb]["ref_id"], "file": items[nb]["file"],
                     "gap_mm": round(gap, 3),
                     "point_mm": [round(float(v), 2) for v in p],
                     "dist_to_servo_mm": round(float(np.linalg.norm(p - servo_pt)), 2)}
                    for nb, p, gap in hits]

        drive = None
        conf = "no_chain"
        surf_gap = None
        driver_part = None
        if skin_ref is not None and chain:
            # 沿链顺序找“最后一个贴近皮肤”的执行点：保持舵机->曲柄->连杆->皮肤方向
            last = None
            for nb, _ in chain:
                fp = front(items[nb]["pts"])
                if fp is None:
                    continue
                got = best_on(fp)
                if got is not None and got[1] <= 45.0:
                    last = (got, items[nb]["file"])
            if last is not None:
                (p, gap_here, j), driver_part = last
                drive = [round(float(v), 2) for v in skin_ref[j]]
                surf_gap = round(gap_here, 2)
                conf = "chain_end_on_skin"
            else:
                fp = front(items[chain[-1][0]]["pts"])
                if fp is not None:
                    p, gap_here, j = best_on(fp)
                    drive = [round(float(v), 2) for v in p]
                    surf_gap = round(gap_here, 2)
                    conf = "chain_end_off_skin"
                    driver_part = items[chain[-1][0]]["file"]
        out[str(sid)] = {
            "ref": ref,
            "servo_t_mm": [round(float(v), 2) for v in servo_pt],
            "owner": items[idx]["owner_model"],
            "servo_file": items[idx]["file"],
            "chain": rows,
            "skin_hits": hit_rows,
            "driver_point_mm": drive,
            "surface_gap_mm": surf_gap,
            "driver_part": driver_part,
            "confidence": conf,
        }

    (GEO / "servo_drive_chains.json").write_text(
        json.dumps({"servos": out}, ensure_ascii=False, indent=1), encoding="utf-8")

    for sid in range(1, 34):
        r = out[str(sid)]
        if r.get("status"):
            print("M%02d  NO_CAD" % sid)
            continue
        print("M%02d %-8s chain=%2d %-20s %-38s %s gap=%s" % (
            sid, r["owner"][:8], len(r["chain"]), r["confidence"],
            (r.get("driver_part") or "-")[:38], r["driver_point_mm"], r["surface_gap_mm"]))
    ok = sum(1 for r in out.values() if r.get("confidence") == "chain_end_on_skin")
    off = sum(1 for r in out.values() if r.get("confidence") == "chain_end_off_skin")
    print("SUMMARY on_skin=%d off_skin=%d none=%d" % (ok, off, 32 - ok - off))


if __name__ == "__main__":
    main()
