import bpy
import json
from pathlib import Path

ROOT = Path(r"C:\file\project\Morpheus-Software")
arm = bpy.data.objects.get("Feace_Rig.001")
if arm is None:
    raise RuntimeError("Feace_Rig.001 not found")


def plain(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if hasattr(value, "keys"):
        return {str(k): plain(value[k]) for k in value.keys()}
    try:
        return [plain(v) for v in value]
    except TypeError:
        return str(value)


bones = {}
for b in arm.data.bones:
    rec = {
        "head_local": list(b.head_local),
        "tail_local": list(b.tail_local),
        "parent": b.parent.name if b.parent else None,
        "use_deform": b.use_deform,
        "props": {k: plain(b[k]) for k in b.keys()},
    }
    try:
        ss = b.servo_settings
        rec["servo_settings_runtime"] = {
            p.identifier: plain(getattr(ss, p.identifier))
            for p in ss.bl_rna.properties
            if p.identifier not in {"rna_type"}
        }
    except Exception as exc:
        rec["servo_settings_error"] = str(exc)
    bones[b.name] = rec

out = {
    "file": bpy.data.filepath,
    "bone_count": len(bones),
    "bones": bones,
    "has_servo_settings_type": hasattr(bpy.types.Bone, "servo_settings"),
    "objects": {o.name: o.type for o in bpy.data.objects},
    "frame": bpy.context.scene.frame_current,
}
name = "original_disk_" + ("blend1" if bpy.data.filepath.endswith(".blend1") else "blend") + ".json"
path = ROOT / name
path.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
print("ORIGINAL_DISK_AUDIT", path)
print("BONE_COUNT", len(bones), "SERVO_TYPE", out["has_servo_settings_type"])
for name, rec in bones.items():
    if "servo_settings" in rec["props"] or "servo_settings_runtime" in rec:
        print(name, json.dumps(rec["props"].get("servo_settings"), ensure_ascii=False), rec.get("servo_settings_runtime"))
