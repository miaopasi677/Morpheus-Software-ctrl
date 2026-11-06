"""Turn decoded SolidWorks CAD into (a) real part dimensions and (b) 1:1
matching of the 33 Morpheus PWM channels to physical servo instances.

Inputs (all produced locally, no SolidWorks needed):
    cad_geometry/stl/*.stl                 from the sldprt Node decoder
    cad_geometry/servo_instances.json     from parse_cad_assembly.py
    servo_geometry.py                     the previous estimated layout

Outputs:
    cad_geometry/part_dimensions.json     121 parts, exact mm bounding boxes
    cad_geometry/servo_id_mapping.json    33 channels -> CAD servo instance

Coordinate systems
------------------
CAD world (SolidWorks assembly, metres -> mm here):
    +X = towards the BACK of the head (-X = face/front)
    +Y = up
    +Z = the subject's LEFT  (shell_left_ears is at Z=+82.55 mm)
Face coordinates used by servo_geometry.py (origin = midpoint of the eyes):
    +X = subject's left, +Y = back of head, +Z = up
    face_x = Z_world - 1.0
    face_y = X_world + 96.84
    face_z = Y_world - 285.87
The constants come from eye_camera_eyeball at (-96.84, 285.87, -34/+36).
"""
from __future__ import annotations

import json
import pathlib
import struct
import sys

ROOT = pathlib.Path(__file__).resolve().parent
GEO = ROOT / "cad_geometry"
sys.path.insert(0, str(ROOT))
import servo_geometry as sg  # noqa: E402

# eye midpoint in SolidWorks assembly world coordinates (mm)
EYE_MID_WORLD = (-96.84, 285.87, 1.0)


def world_to_face(t):
    xw, yw, zw = t
    return (round(zw - EYE_MID_WORLD[2], 3),
            round(xw - EYE_MID_WORLD[0], 3),
            round(yw - EYE_MID_WORLD[1], 3))


def stl_bbox(path):
    b = pathlib.Path(path).read_bytes()
    if len(b) < 84:
        return None
    if b[:5] == b"solid":
        xs, ys, zs = [], [], []
        for line in b.decode("latin1").splitlines():
            f = line.strip().split()
            if len(f) == 4 and f[0] == "vertex":
                xs.append(float(f[1])); ys.append(float(f[2])); zs.append(float(f[3]))
        if not xs:
            return None
    else:
        n = struct.unpack_from("<I", b, 80)[0]
        xs, ys, zs = [], [], []
        off = 84
        for _ in range(n):
            v = struct.unpack_from("<12f", b, off)
            xs.extend((v[3], v[6], v[9]))
            ys.extend((v[4], v[7], v[10]))
            zs.extend((v[5], v[8], v[11]))
            off += 50
    lo = [min(xs), min(ys), min(zs)]
    hi = [max(xs), max(ys), max(zs)]
    return {
        "triangles": len(xs) // 3,
        "min_mm": [round(v, 3) for v in lo],
        "max_mm": [round(v, 3) for v in hi],
        "size_mm": [round(hi[i] - lo[i], 3) for i in range(3)],
        "center_mm": [round((lo[i] + hi[i]) / 2, 3) for i in range(3)],
    }


OWNER_TO_MODULES = {
    "EYEBROW": {"tendon", "eyebrow"},
    "eye": {"eye"},
    "mouth assembly": {"mouth"},
    "neck": {"neck"},
}


def group_key(module):
    for owner, mods in OWNER_TO_MODULES.items():
        if module in mods:
            return owner
    return "other"


def match_group(ids, instances, weights=(2.0, 0.4, 1.0)):
    """One-to-one greedy match on SHAPE (centroid-removed) to ignore the
    systematic offset between the old estimate and the real CAD origin.

    Left/right is the strongest signal: face_x is + for the subject's left
    in both systems. Centre channels must stay near the midline, and lateral
    channels must not flip side or collapse onto the midline.
    """
    old = {sid: sg.SERVO_MM[sid]["xyz_mm"] for sid in ids}
    cad = {i: world_to_face(r["world_t_mm"]) for i, r in enumerate(instances)}
    if not old or not cad:
        return {}
    oc = [sum(old[s][k] for s in ids) / len(ids) for k in range(3)]
    cc = [sum(cad[i][k] for i in cad) / len(cad) for k in range(3)]
    cand = []
    for sid in ids:
        for i, _ in enumerate(instances):
            d = ((old[sid][0] - oc[0] - (cad[i][0] - cc[0])) ** 2 * weights[0]
                 + (old[sid][1] - oc[1] - (cad[i][1] - cc[1])) ** 2 * weights[1]
                 + (old[sid][2] - oc[2] - (cad[i][2] - cc[2])) ** 2 * weights[2]) ** 0.5
            ox, cx = old[sid][0], cad[i][0]
            if abs(ox) <= 3 and abs(cx) > 15:      # centre channel -> lateral servo
                d += 60.0
            if abs(ox) > 8:                        # lateral channel
                if abs(cx) < 5:
                    d += 50.0                      # must not take a centre servo
                if ox * cx < 0:
                    d += 150.0                     # must not flip left/right
            cand.append((d, sid, i))
    cand.sort()
    out, used_s, used_i = {}, set(), set()
    for d, sid, i in cand:
        if sid in used_s or i in used_i:
            continue
        out[sid] = {"instance_index": i, "shape_distance_mm": round(d, 2)}
        used_s.add(sid); used_i.add(i)
    return out


def main():
    # ---- (a) exact part dimensions -------------------------------------
    parts = {}
    for p in sorted((GEO / "stl").glob("*.stl")):
        if p.name == "FULL_ASSEMBLY.stl":
            continue
        try:
            bb = stl_bbox(p)
            if bb:
                parts[p.stem] = bb
        except Exception as e:
            parts[p.stem] = {"error": str(e)}
    (GEO / "part_dimensions.json").write_text(json.dumps({
        "source": "SolidWorks 2022 SLDPRT -> sldprt decoder -> STL",
        "units": "mm", "count": len(parts), "parts": parts,
    }, ensure_ascii=False, indent=1), encoding="utf-8")

    # ---- (b) 33 channels -> CAD servo instances ------------------------
    data = json.loads((GEO / "servo_instances.json").read_text(encoding="utf-8"))
    servos = data["servos"]
    by_owner = {k: [r for r in servos if r["owner_model"] == k]
                for k in OWNER_TO_MODULES}
    groups = {}
    for owner, mods in OWNER_TO_MODULES.items():
        groups[owner] = [sid for sid in sg.SERVO_MM
                        if sg.SERVO_MM[sid]["module"] in mods]
    by_id, unmatched = {}, []
    for owner, ids in groups.items():
        m = match_group(ids, by_owner[owner])
        for sid, hit in m.items():
            r = by_owner[owner][hit["instance_index"]]
            by_id[sid] = {
                "id": sid, "module": sg.SERVO_MM[sid]["module"],
                "old_xyz_face_mm": list(sg.SERVO_MM[sid]["xyz_mm"]),
                "cad_xyz_face_mm": list(world_to_face(r["world_t_mm"])),
                "cad_world_mm": r["world_t_mm"],
                "cad_part": r["name"] + " / " + r["file"],
                "cad_owner": r["owner_model"],
                "cad_ref_id": r["ref_id"],
                "shape_distance_mm": hit["shape_distance_mm"],
                "confidence": "cad_measured",
            }
        unmatched.extend(s for s in ids if s not in m)
    for sid in unmatched:
        by_id[sid] = {
            "id": sid, "module": sg.SERVO_MM[sid]["module"],
            "old_xyz_face_mm": list(sg.SERVO_MM[sid]["xyz_mm"]),
            "cad_xyz_face_mm": None,
            "cad_part": None, "cad_owner": None, "cad_ref_id": None,
            "shape_distance_mm": None,
            "confidence": "estimated_no_cad_instance",
        }
    (GEO / "servo_id_mapping.json").write_text(json.dumps({
        "source": "SolidWorks 2022 full_assembly.SLDASM / swXmlContents/COMPINSTANCETREE",
        "units": "mm",
        "eye_mid_world_mm": list(EYE_MID_WORLD),
        "transform": "face_x=Z-1, face_y=X+96.84, face_z=Y-285.87",
        "cad_servo_instances": len(servos),
        "matched": len(servos), "unmatched_ids": unmatched,
        "by_id": by_id,
    }, ensure_ascii=False, indent=1), encoding="utf-8")

    print("part dimensions: %d" % len(parts))
    print("CAD servo instances: %d ; matched channels: %d ; unmatched: %s" %
          (len(servos), len(servos), unmatched))
    print("\n=== module planes from CAD (face Z mm) ===")
    for owner in OWNER_TO_MODULES:
        zs = [r["cad_xyz_face_mm"][2] for r in by_id.values()
              if r["cad_owner"] == owner and r["cad_xyz_face_mm"]]
        if zs:
            print("  %-16s n=%2d  Z %8.1f .. %8.1f mm" %
                  (owner, len(zs), min(zs), max(zs)))
    print("\n=== sample mapping ===")
    for sid in (1, 3, 5, 8, 23, 31):
        r = by_id[sid]
        print("  M%02d %-8s %s -> %s  d=%s %s" %
              (sid, r["module"], str(r["old_xyz_face_mm"]),
               str(r["cad_xyz_face_mm"]), r["shape_distance_mm"], r["cad_part"]))
    return by_id


if __name__ == "__main__":
    main()
