"""从 CAD 接触簇提取 33 路机构的真实铰链点 / 固定铰 / 模块平面，供 Blender 建机构骨。

不做任何运动求解——只提取几何（用户方向：模型建准，运动靠 Blender 原生约束）。

每路输出：
  type        cable / rack / linkage / pushrod / eye_rev / neck_rev / jaw_rev / open
  servo_axis  舵机输出轴真实位置（servo↔曲柄接触簇质心，比零件原点准）
  plane       模块平面 {origin, normal, u, v}（铰链轴 PCA 方向中位数）
  joints      相邻传动件接触簇 = 活动铰 {a, b, world_mm, p2d, axis}
  grounds     传动件↔结构件接触簇 = 固定铰 {part, world_mm, p2d, axis}
  links       链上零件顺序 + 各自铰链局部坐标
  crank_r     曲柄半径（servo_axis -> 第一活动铰，平面内）
  anchor      表皮驱动点 {world_mm, p2d, part}
  confidence  提取完整度标注

运行（Blender 自带 numpy）：
  blender --background --python extract_linkage_model.py
"""
import json
import pathlib
import sys
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import trace_drive_chains as tdc

GEO = ROOT / "cad_geometry"
VOX = 0.6


def contact_clusters(pa, pb, thresh=1.2, max_radius=6.0, min_pts=15):
    """接触簇 = 体素网格连通域。返回 [{cen, axis, n, radius}]，只留紧凑簇（铰链环）。"""
    if pa is None or pb is None or not len(pa) or not len(pb):
        return []
    mids = []
    step = 300
    t2 = thresh * thresh
    for i in range(0, len(pa), step):
        a = pa[i:i + step]
        d2 = ((a[:, None, :] - pb[None, :, :]) ** 2).sum(-1)
        ia, ib = np.nonzero(d2 <= t2)
        if len(ia):
            mids.append((a[ia] + pb[ib]) * 0.5)
    if not mids:
        return []
    m = np.vstack(mids)
    keys = np.floor(m / VOX).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    nv = len(uniq)
    cnt = np.bincount(inv, minlength=nv).astype(float)
    sums = np.zeros((nv, 3))
    np.add.at(sums, inv, m)
    vcen = sums / cnt[:, None]
    # union-find over 27-neighborhood
    pos = {tuple(k): i for i, k in enumerate(uniq)}
    parent = list(range(nv))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for i, k in enumerate(uniq):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    j = pos.get((k[0] + dx, k[1] + dy, k[2] + dz))
                    if j is not None and j > i:
                        ri, rj = find(i), find(j)
                        if ri != rj:
                            parent[rj] = ri
    groups = {}
    for i in range(nv):
        groups.setdefault(find(i), []).append(i)
    out = []
    for g in groups.values():
        n = float(cnt[g].sum())
        if n < min_pts:
            continue
        cen = (vcen[g] * cnt[g][:, None]).sum(0) / n
        rad = float(np.sqrt(((vcen[g] - cen) ** 2).sum(-1).max()) + VOX)
        if rad > max_radius:
            continue                      # 长线接触（钢丝贴钢丝）不是铰链
        w = cnt[g]
        cov = ((vcen[g] - cen).T * w) @ (vcen[g] - cen) / n
        _, v = np.linalg.eigh(cov)
        out.append({"cen": cen, "axis": v[:, 0], "n": int(n), "radius": round(rad, 2)})
    out.sort(key=lambda r: -r["n"])
    return out


def best_joint(pa, pb):
    for th in (1.2, 2.0, 3.0):
        cl = contact_clusters(pa, pb, thresh=th)
        if cl:
            return cl[0], th
    return None, None


def plane_basis(normal):
    n = np.array(normal, float)
    n /= np.linalg.norm(n)
    ref = np.array((0.0, 1.0, 0.0)) if abs(n[1]) < 0.9 else np.array((1.0, 0.0, 0.0))
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return n, u, v


def main():
    items = tdc.load_world_points()
    by_ref = {it["ref_id"]: i for i, it in enumerate(items)}
    chains = json.loads((GEO / "servo_drive_chains.json").read_text(encoding="utf-8"))["servos"]
    structs = [i for i, it in enumerate(items) if it["klass"] == "struct" and it["pts"] is not None]

    out = {}
    for sid in range(1, 34):
        r = chains[str(sid)]
        rec = {"id": sid, "owner": r.get("owner"), "chain_files":
               [x["file"] for x in r.get("chain", [])], "joints": [], "grounds": [],
               "notes": []}
        idxs = [by_ref.get(x) for x in ([r["ref"]] + [c["ref"] for c in r.get("chain", [])])]
        idxs = [i for i in idxs if i is not None]
        if not idxs or items[idxs[0]]["klass"] != "servo":
            rec.update(type="open", confidence="no_cad_servo",
                       servo_axis_mm=r.get("servo_t_mm"), anchor=None)
            out[str(sid)] = rec
            continue

        servo_axis = np.array(items[idxs[0]]["world_t_mm"], float)
        # 相邻活动铰
        axes = []
        for a, b in zip(idxs, idxs[1:]):
            j, th = best_joint(items[a]["pts"], items[b]["pts"])
            if j is None:
                rec["notes"].append("no_contact:%s|%s" % (items[a]["file"], items[b]["file"]))
                rec["joints"].append({"a": items[a]["file"], "b": items[b]["file"],
                                      "world_mm": None, "thresh": None})
                continue
            if a == idxs[0]:
                servo_axis = j["cen"]          # 输出轴真实位置
            axes.append(j["axis"])
            rec["joints"].append({"a": items[a]["file"], "b": items[b]["file"],
                                  "world_mm": [round(float(x), 2) for x in j["cen"]],
                                  "axis": [round(float(x), 3) for x in j["axis"]],
                                  "n": j["n"], "radius": j["radius"], "thresh": th})
        # 固定铰：传动件 ↔ 结构件
        for i in idxs[1:]:
            for s in structs:
                if tdc.bbox_gap(items[i]["bbox"], items[s]["bbox"]) > 6.0:
                    continue
                cl = contact_clusters(items[i]["pts"], items[s]["pts"], thresh=1.5)
                if not cl:
                    cl = contact_clusters(items[i]["pts"], items[s]["pts"], thresh=2.5)
                if cl:
                    j = cl[0]
                    axes.append(j["axis"])
                    rec["grounds"].append({"part": items[i]["file"], "struct": items[s]["file"],
                                           "world_mm": [round(float(x), 2) for x in j["cen"]],
                                           "axis": [round(float(x), 3) for x in j["axis"]],
                                           "n": j["n"], "radius": j["radius"]})
        # 模块平面：铰链轴一致性投票
        normal = None
        if axes:
            axes = np.array(axes)
            ref = axes[0]
            agree = axes[np.abs(axes @ ref) > 0.9]
            if len(agree) >= 2:
                normal = agree.mean(0)
                normal *= np.sign(normal @ ref)
        if normal is None:
            normal = (0, 1, 0) if r.get("owner") == "neck" else (0, 0, 1)
            rec["notes"].append("plane_default")
        n, u, v = plane_basis(normal)
        origin = servo_axis

        def p2d(p):
            d = np.array(p, float) - origin
            return [round(float(d @ u), 2), round(float(d @ v), 2)]

        for j in rec["joints"]:
            j["p2d"] = p2d(j["world_mm"]) if j["world_mm"] else None
        for g in rec["grounds"]:
            g["p2d"] = p2d(g["world_mm"])
        rec["plane"] = {"origin": [round(float(x), 2) for x in origin],
                        "normal": [round(float(x), 3) for x in n],
                        "u": [round(float(x), 3) for x in u],
                        "v": [round(float(x), 3) for x in v]}
        rec["servo_axis_mm"] = [round(float(x), 2) for x in servo_axis]

        # 曲柄半径 = servo_axis -> 曲柄销（第二个铰：曲柄↔下一级），不是同轴的第一个铰
        j1 = next((j for j in rec["joints"][1:] if j["p2d"]), None)
        if j1 is None:
            j1 = next((j for j in rec["joints"] if j["p2d"]), None)
        rec["crank_r_mm"] = round(float(np.hypot(*j1["p2d"])), 2) if j1 else None
        rec["crank_pin_mm"] = j1["world_mm"] if j1 else None

        # 表皮锚点
        anchor_part = rec["chain_files"][-1] if rec["chain_files"] else None
        aw = r.get("driver_point_mm")
        if aw is None and r.get("skin_hits"):
            aw = r["skin_hits"][0]["point_mm"]
            anchor_part = r["skin_hits"][0]["file"]
        rec["anchor"] = ({"world_mm": aw, "p2d": p2d(aw), "part": anchor_part}
                         if aw else None)

        # 类型分类
        low = " ".join(rec["chain_files"]).lower()
        js_ok = [j for j in rec["joints"] if j["p2d"]]
        has_wire = any("wire" in f.lower() for f in rec["chain_files"])
        output_grounded = bool(rec["chain_files"]) and any(
            g["part"] == rec["chain_files"][-1] for g in rec["grounds"])
        if sid in (7, 24):
            typ = "eye_rev"
        elif sid in (31, 32, 33):
            typ = "neck_rev"
        elif sid == 23 and any("jaw" in f.lower() for f in rec["chain_files"]):
            typ = "jaw_rev"
        elif "rack" in low:
            typ = "rack"
        elif rec["chain_files"] and any(k in rec["chain_files"][-1].lower()
                                        for k in ("line", "wire")) and "mechanism" not in low:
            typ = "cable"
        elif any(k in low for k in ("shanks", "threaded rod", "ball_tie")):
            typ = "pushrod"
        elif rec["grounds"] and len(js_ok) >= 2 and (output_grounded or not has_wire):
            # 含钢丝的链只有当输出件本身接地时才作刚性四连杆等效（钢丝张紧段=连杆）；
            # 否则钢丝末端是自由表皮锚点 -> 线驱平动（M23 下颌/唇拉线）。
            typ = "linkage"
        elif any(k in low for k in ("line", "wire")):
            typ = "cable"
        else:
            typ = "open"
        rec["type"] = typ
        # 真实驱动方向（世界坐标单位向量）：平动类沿它走，open 类作一阶近似
        drive_dir = None
        if typ == "rack":
            # 轨轴 = 齿条零件 PCA 主轴（与表皮锚点无关，无锚点也要有方向）
            ri = next((i for i in idxs if "rack" in items[i]["file"].lower()), None)
            if ri is not None and items[ri]["pts"] is not None:
                pp = items[ri]["pts"]
                cov = np.cov((pp - pp.mean(0)).T)
                _, vv = np.linalg.eigh(cov)
                ax = vv[:, -1]
                d2 = np.array((ax @ u, ax @ v))
                if np.linalg.norm(d2) > 1e-6:
                    d2 /= np.linalg.norm(d2)
                    drive_dir = u * d2[0] + v * d2[1]
        aw = np.array(aw, float) if aw else None
        if aw is not None:
            if typ == "cable" and rec["joints"] and rec["joints"][-1]["world_mm"]:
                guide = np.array(rec["joints"][-1]["world_mm"], float)
                drive_dir = guide - aw
            elif typ == "pushrod" and rec["crank_pin_mm"]:
                drive_dir = aw - np.array(rec["crank_pin_mm"], float)
            if drive_dir is None:
                drive_dir = servo_axis - aw        # 一阶：拉向舵机侧
        if drive_dir is not None and np.linalg.norm(drive_dir) > 1e-9:
            drive_dir = drive_dir / np.linalg.norm(drive_dir)
            rec["drive_dir_world"] = [round(float(x), 4) for x in drive_dir]
        rec["confidence"] = ("extracted" if typ == "linkage" and len(js_ok) == len(rec["joints"])
                             else "partial" if js_ok or typ in ("cable", "rack", "eye_rev", "neck_rev", "jaw_rev")
                             else "low")
        out[str(sid)] = rec
        print("M%02d %-9s joints=%d/%d grounds=%d crank_r=%s conf=%s %s" % (
            sid, typ, len(js_ok), len(rec["joints"]), len(rec["grounds"]),
            rec["crank_r_mm"], rec["confidence"], ";".join(rec["notes"])[:60]))

    # M16 齿条：曲柄接触缺失 + 默认平面，PCA 主轴抓到了齿条体长轴而非滑动轨轴。
    # M13/M16 是同族镜像齿条 -> 借 M13 的轨轴（CAD z 镜像）；符号按 PWM 表已校准的
    # 功能方向取（mouthLowerDown 正激活 -> 下移），硬件若反向改 servo_map direction。
    r13, r16 = out.get("13"), out.get("16")
    if r13 and r16 and r16["type"] == "rack" and (r16.get("crank_r_mm") or 0) < 0.1 \
            and r13.get("drive_dir_world"):
        d = r13["drive_dir_world"]
        r16["drive_dir_world"] = [round(-d[0], 4), round(-d[1], 4), round(d[2], 4)]
        r16["notes"].append("rail_from_mirror:M13")

    (GEO / "linkage_model.json").write_text(
        json.dumps({"servos": out}, ensure_ascii=False, indent=1), encoding="utf-8")
    print("EXTRACT_DONE types=", json.dumps(
        {t: sum(1 for r in out.values() if r["type"] == t) for t in
         sorted({r["type"] for r in out.values()})}))


if __name__ == "__main__":
    main()