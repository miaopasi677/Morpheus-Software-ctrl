"""探针：接触簇能否作为铰链点。M04 眉通道 + M14 五连杆 + M16 齿条。"""
import json
import pathlib
import sys
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
import trace_drive_chains as tdc

items = tdc.load_world_points()
by_ref = {it["ref_id"]: i for i, it in enumerate(items)}
chains = json.loads((ROOT / "cad_geometry" / "servo_drive_chains.json").read_text(
    encoding="utf-8"))["servos"]


def clusters(pa, pb, thresh=1.2, gap=3.0):
    """两零件点云接触簇：返回 (质心, 轴向PCA最小分量, 点数, 簇半径)。"""
    if pa is None or pb is None:
        return []
    mids = []
    step = 300
    for i in range(0, len(pa), step):
        a = pa[i:i + step]
        d2 = ((a[:, None, :] - pb[None, :, :]) ** 2).sum(-1)
        ia, ib = np.nonzero(d2 <= thresh * thresh)
        if len(ia):
            mids.append((a[ia] + pb[ib]) * 0.5)
    if not mids:
        return []
    m = np.vstack(mids)
    # 贪心连通簇
    used = np.zeros(len(m), bool)
    out = []
    for s in range(len(m)):
        if used[s]:
            continue
        stack = [s]
        used[s] = True
        comp = [s]
        while stack:
            u = stack.pop()
            d2 = ((m - m[u]) ** 2).sum(-1)
            nb = np.nonzero((d2 <= gap * gap) & ~used)[0]
            used[nb] = True
            stack.extend(nb.tolist())
            comp.extend(nb.tolist())
        c = m[comp]
        cen = c.mean(0)
        cov = np.cov((c - cen).T) if len(c) > 3 else np.eye(3)
        w, v = np.linalg.eigh(cov)
        axis = v[:, 0]
        rad = float(np.sqrt(((c - cen) ** 2).sum(-1).max()))
        out.append((cen, axis, len(comp), rad))
    out.sort(key=lambda x: -x[2])
    return out


for sid in ("4", "14", "16", "23"):
    r = chains[sid]
    print("=== M%s ===" % sid)
    refs = [r["ref"]] + [x["ref"] for x in r["chain"]]
    idxs = [by_ref.get(x) for x in refs]
    for a, b in zip(idxs, idxs[1:]):
        if a is None or b is None:
            continue
        cl = clusters(items[a]["pts"], items[b]["pts"])
        print("  %s <-> %s" % (items[a]["file"][:34], items[b]["file"][:34]))
        for cen, axis, n, rad in cl[:3]:
            print("     joint n=%4d r=%5.2f cen=(%7.2f,%7.2f,%7.2f) axis=(%5.2f,%5.2f,%5.2f)"
                  % (n, rad, cen[0], cen[1], cen[2], axis[0], axis[1], axis[2]))
    # 固定铰：传动件与结构件接触
    for i in idxs[1:]:
        if i is None:
            continue
        for j, it in enumerate(items):
            if it["klass"] != "struct" or it["pts"] is None:
                continue
            if tdc.bbox_gap(items[i]["bbox"], it["bbox"]) > 1.5:
                continue
            cl = clusters(items[i]["pts"], it["pts"])
            if cl:
                cen, axis, n, rad = cl[0]
                print("  GROUND? %s <-> %s n=%d cen=(%.1f,%.1f,%.1f) axis=(%.2f,%.2f,%.2f)"
                      % (items[i]["file"][:30], it["file"][:30], n,
                         cen[0], cen[1], cen[2], axis[0], axis[1], axis[2]))