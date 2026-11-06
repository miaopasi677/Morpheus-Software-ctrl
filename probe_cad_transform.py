"""校验 STL 局部点 -> world 的旋转约定，并统计可用的每实例点数。

在 Blender 的 python 下运行（自带 numpy）：
  blender --background --python probe_cad_transform.py
"""

import json
import struct
import pathlib
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent
GEO = ROOT / "cad_geometry"
STL = GEO / "stl"


def read_binary_stl(path):
    data = pathlib.Path(path).read_bytes()
    if len(data) < 84:
        return None
    n = struct.unpack("<I", data[80:84])[0]
    need = 84 + n * 50
    if need > len(data):
        n = (len(data) - 84) // 50
    if n <= 0:
        return None
    raw = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = raw[:, 12:48].copy().view("<f4").reshape(n, 3, 3)
    return tri.astype(np.float64).reshape(-1, 3)


def stl_for(item):
    base = item["file"].rsplit(".", 1)[0]
    owner = item["owner_model"]
    cands = [
        STL / (owner + "__" + base + ".stl"),
        STL / (base + ".stl"),
    ]
    for c in cands:
        if c.exists():
            return c
    low = base.lower()
    for p in STL.glob("*.stl"):
        name = p.stem.split("__")[-1].lower()
        if name == low:
            return p
    return None


def main():
    inst = json.loads((GEO / "assembly_instances.json").read_text(encoding="utf-8"))["instances"]
    cache = {}
    rows = []
    for it in inst:
        p = stl_for(it)
        if p is None:
            rows.append((it, None, "no_stl"))
            continue
        if p not in cache:
            cache[p] = read_binary_stl(p)
        pts = cache[p]
        if pts is None or len(pts) == 0:
            rows.append((it, None, "empty_stl"))
            continue
        R = np.array(it["world_R"], dtype=np.float64)
        t = np.array(it["world_t_mm"], dtype=np.float64)
        bb = it["world_bbox"]
        best = None
        for tag, world in (("R@l+t", pts @ R.T + t), ("l@R+t", pts @ R + t)):
            lo, hi = world.min(0), world.max(0)
            # 点云必须落在 world_bbox 内；超出量才是真正的错误
            over = float(max(0.0, (np.array(bb["min_mm"]) - lo).max())
                         + max(0.0, (hi - np.array(bb["max_mm"])).max()))
            best = (over, tag, world) if best is None or over < best[0] else best
        rows.append((it, best, "ok"))

    ok = [r for r in rows if r[2] == "ok"]
    print("INSTANCES", len(rows), "WITH_STL", len(ok))
    tags = {}
    errs = []
    for it, best, _ in ok:
        tags[(it["owner_model"], best[1])] = tags.get((it["owner_model"], best[1]), 0) + 1
        errs.append(best[0])
    print("CONVENTION", tags)
    errs = np.array(errs)
    print("OUTSIDE_MM max=%.3f p95=%.3f median=%.3f" % (errs.max(), np.percentile(errs, 95), np.median(errs)))
    worst = sorted(ok, key=lambda r: -r[1][0])[:8]
    for it, best, _ in worst:
        print("  WORST %.2fmm %-52s owner=%s" % (best[0], it["file"][:52], it["owner_model"]))
    total = sum(len(cache[stl_for(it)]) for it, _, s in ok if stl_for(it) in cache)
    print("TOTAL_POINTS", total)


if __name__ == "__main__":
    main()
