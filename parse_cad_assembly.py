"""Parse SolidWorks COMPINSTANCETREE.xml -> exact assembly positions in millimetres.

The XML lives inside a modern (SW2015+) .SLDASM as the stream
``swXmlContents/COMPINSTANCETREE``. Extract it with
``cad_geometry/extract_asm_stream.js`` (requires the open-source
``solidworks-converters`` Node decoder). No SolidWorks installation needed.

Every <swReference> carries ``swTransform``: 16 floats, row-major 4x4 with
the translation in the LAST row (row-vector convention, p' = p @ M).
Units are METRES. Sub-assembly models own their child references, so the
world matrix is M_local @ M_parent.

Outputs:
    cad_geometry/assembly_instances.json  all 280 component instances
    cad_geometry/servo_instances.json    servo motors only (exact XYZ mm)
"""
from __future__ import annotations

import json
import pathlib
import re
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent
GEO = ROOT / "cad_geometry"
XML = GEO / "COMPINSTANCETREE.xml"

L = lambda tag: tag.split("}")[-1]
I4 = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def parse_matrix(text):
    v = [float(x) for x in text.split()]
    if len(v) != 16:
        raise ValueError("swTransform has %d floats" % len(v))
    return [v[i * 4:(i + 1) * 4] for i in range(4)]


def translation(m):
    return [round(m[3][i] * 1000.0, 4) for i in range(3)]


def rotation(m):
    return [[round(m[i][j], 6) for j in range(3)] for i in range(3)]


def transform_point(m, p):
    """Row-vector convention: p' = p @ M (p in metres)."""
    v = [p[0], p[1], p[2], 1.0]
    out = [sum(v[k] * m[k][j] for k in range(4)) for j in range(4)]
    return [out[i] * 1000.0 for i in range(3)]


def corners(box):
    x0, y0, z0, x1, y1, z1 = box
    return [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]


SERVO_RE = re.compile(r"MG90S|GUOHUA|A0090|ks3518|KS3518", re.I)


def main():
    root = ET.fromstring(XML.read_bytes().decode("utf-8", "replace"))
    parents = {}
    for node in root.iter():
        for ch in node:
            parents[ch] = node

    files = {f.get("id"): f.attrib for f in root.iter() if L(f.tag) == "swFile"}
    models = {m.get("id"): m for m in root.iter() if L(m.tag) == "swModel"}
    refs = [r for r in root.iter() if L(r.tag) == "swReference"]

    def model_info(mid):
        m = models.get(mid, {})
        f = files.get(m.get("swFileRef") if hasattr(m, "get") else None, {})
        path = f.get("swPath", "")
        bbox = None
        if hasattr(m, "get") and m.get("swBoundingBox"):
            try:
                bbox = [float(v) for v in m.get("swBoundingBox").split()]
            except ValueError:
                pass
        return {
            "model_id": mid,
            "model_name": m.get("swName") if hasattr(m, "get") else "",
            "file": path.replace("\\", "/").split("/")[-1],
            "doc_type": f.get("swDocType", ""),
            "bbox_m": bbox,
        }

    # sub-assembly references live directly under the full_assembly model
    asm_models = {mid for mid, m in models.items()
                  if files.get(m.get("swFileRef"), {}).get("swDocType") == "ASSEMBLY"}
    parent_matrix = {}
    for r in refs:
        if r.get("swModelRef") in asm_models:
            owner = parents.get(r)
            if owner is not None and owner.get("id") == "2":  # full_assembly model
                parent_matrix[r.get("swModelRef")] = parse_matrix(r.get("swTransform"))

    instances = []
    for r in refs:
        owner = parents.get(r)
        owner_id = owner.get("id") if owner is not None else None
        mi = model_info(r.get("swModelRef"))
        M = parse_matrix(r.get("swTransform"))
        P = parent_matrix.get(owner_id, I4)
        W = matmul(M, P)
        bbox_world = None
        if mi["bbox_m"]:
            pts = [transform_point(W, p) for p in corners(mi["bbox_m"])]
            bbox_world = {
                "min_mm": [round(min(p[i] for p in pts), 3) for i in range(3)],
                "max_mm": [round(max(p[i] for p in pts), 3) for i in range(3)],
                "center_mm": [round((min(p[i] for p in pts) + max(p[i] for p in pts)) / 2, 3)
                              for i in range(3)],
            }
        instances.append({
            "ref_id": r.get("id"),
            "name": r.get("swName"),
            "instance": r.get("swReferenceNumber"),
            "owner_model_id": owner_id,
            "owner_model": model_info(owner_id)["model_name"] if owner_id in models else "",
            **mi,
            "local_t_mm": translation(M),
            "local_R": rotation(M),
            "world_t_mm": translation(W),
            "world_R": rotation(W),
            "world_bbox": bbox_world,
        })

    servos = [x for x in instances if SERVO_RE.search(x["name"] or "")
              or SERVO_RE.search(x["file"] or "")]

    (GEO / "assembly_instances.json").write_text(
        json.dumps({"source": "swXmlContents/COMPINSTANCETREE (SolidWorks 2022)",
                    "units": "mm", "count": len(instances),
                    "instances": instances}, ensure_ascii=False, indent=1),
        encoding="utf-8")
    (GEO / "servo_instances.json").write_text(
        json.dumps({"source": "swXmlContents/COMPINSTANCETREE (SolidWorks 2022)",
                    "units": "mm", "count": len(servos),
                    "servos": servos}, ensure_ascii=False, indent=1),
        encoding="utf-8")

    print("instances: %d   servo instances: %d" % (len(instances), len(servos)))
    print("\n=== sub-assembly world origins (mm) ===")
    for mid, M in sorted(parent_matrix.items()):
        print("  %-18s t=%s" % (model_info(mid)["model_name"], translation(M)))
    print("\n=== servo instances (name / owner / local t / world t / file) ===")
    for s in servos:
        print("  %-24s %-16s local=%-24s world=%-24s %s" %
              (s["name"], s["owner_model"], str(s["local_t_mm"]),
               str(s["world_t_mm"]), s["file"]))
    return instances, servos


if __name__ == "__main__":
    main()
