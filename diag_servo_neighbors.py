"""诊断单个舵机周围的零件距离，用于确定传动链连接阈值。"""

import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import trace_drive_chains as T


def main():
    items = T.load_world_points()
    by_ref = {it["ref_id"]: i for i, it in enumerate(items)}
    mapping = json.loads((T.GEO / "servo_id_mapping.json").read_text(encoding="utf-8"))["by_id"]

    for sid in (1, 3, 5, 8, 12, 23):
        rec = mapping.get(str(sid)) or {}
        idx = by_ref.get(rec.get("cad_ref_id"))
        if idx is None:
            print("M%02d no cad" % sid)
            continue
        s = items[idx]
        sp = np.array(s["world_t_mm"], dtype=np.float64)
        print("=== M%02d %s ref=%s t=%s" % (sid, s["file"], s["ref_id"],
                                            [round(v, 1) for v in sp]))
        rows = []
        for j, it in enumerate(items):
            if j == idx:
                continue
            gap = T.bbox_gap(s["bbox"], it["bbox"])
            if gap > 12:
                continue
            d, pa, pb = T.min_dist(s["pts"], it["pts"])
            ap = T.min_dist(np.array([sp]), it["pts"])[0]
            rows.append((gap, d, ap, it["klass"], it["file"], it["owner_model"]))
        rows.sort(key=lambda r: (r[0], r[1]))
        for gap, d, ap, kl, f, ow in rows[:10]:
            print("   gap=%6.2f pts=%7.2f axis=%7.2f %-7s %-46s %s" % (
                gap, d, ap, kl, f[:46], ow))


if __name__ == "__main__":
    main()
