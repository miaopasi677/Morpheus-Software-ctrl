import bpy
import json
from pathlib import Path

ROOT = Path(r"C:\file\project\Morpheus-Software")


def fc_info(fc):
    info = {"data_path": fc.data_path, "array_index": fc.array_index}
    d = fc.driver
    if d:
        info["driver"] = {
            "type": d.type,
            "expression": d.expression,
            "variables": [
                {
                    "name": v.name,
                    "type": v.type,
                    "targets": [
                        {
                            "id_type": t.id_type,
                            "id": t.id.name if t.id else None,
                            "data_path": t.data_path,
                            "bone_target": t.bone_target,
                            "transform_type": t.transform_type,
                            "transform_space": t.transform_space,
                        }
                        for t in v.targets
                    ],
                }
                for v in d.variables
            ],
        }
    return info


def anim_info(id_block):
    ad = id_block.animation_data
    if not ad:
        return None
    return {
        "action": ad.action.name if ad.action else None,
        "action_slot": ad.action_slot.identifier if ad.action_slot else None,
        "nla": [
            {"name": t.name, "strip": s.name, "action": s.action.name if s.action else None,
             "frame_start": s.frame_start, "frame_end": s.frame_end, "influence": s.influence}
            for t in ad.nla_tracks for s in t.strips
        ],
        "drivers": [fc_info(fc) for fc in ad.drivers],
    }


report = {
    "file": bpy.data.filepath,
    "frame": bpy.context.scene.frame_current,
    "frame_range": [bpy.context.scene.frame_start, bpy.context.scene.frame_end],
    "fps": bpy.context.scene.render.fps,
    "objects": [],
    "armatures": {},
    "actions": [],
    "texts": {},
}
for ob in bpy.data.objects:
    rec = {
        "name": ob.name,
        "type": ob.type,
        "location": list(ob.location),
        "rotation_euler": list(ob.rotation_euler),
        "scale": list(ob.scale),
        "parent": ob.parent.name if ob.parent else None,
        "parent_type": ob.parent_type,
        "parent_bone": ob.parent_bone,
        "hide_viewport": ob.hide_viewport,
        "hide_render": ob.hide_render,
        "animation": anim_info(ob),
        "constraints": [],
        "modifiers": [],
        "custom_props": {k: (list(v) if hasattr(v, "__len__") and not isinstance(v, str) else v)
                         for k, v in ob.items() if k.startswith("morpheus")},
    }
    for c in ob.constraints:
        rec["constraints"].append({
            "name": c.name, "type": c.type, "influence": c.influence, "mute": c.mute,
            "target": c.target.name if hasattr(c, "target") and c.target else None,
            "subtarget": c.subtarget if hasattr(c, "subtarget") else None,
        })
    for m in ob.modifiers:
        rec["modifiers"].append({"name": m.name, "type": m.type, "show_viewport": m.show_viewport})
    if ob.type == "MESH" and ob.data.shape_keys:
        sk = ob.data.shape_keys
        rec["shape_keys"] = {
            "name": sk.name,
            "count": len(sk.key_blocks),
            "animation": anim_info(sk),
            "keys": [{"name": k.name, "value": k.value, "min": k.slider_min, "max": k.slider_max}
                     for k in sk.key_blocks],
        }
    report["objects"].append(rec)
    if ob.type == "ARMATURE":
        bones = []
        for pb in ob.pose.bones:
            b = pb.bone
            bones.append({
                "name": b.name,
                "head_local": list(b.head_local),
                "tail_local": list(b.tail_local),
                "head": list(pb.head),
                "tail": list(pb.tail),
                "parent": b.parent.name if b.parent else None,
                "use_deform": b.use_deform,
                "rotation_mode": pb.rotation_mode,
                "location": list(pb.location),
                "rotation_euler": list(pb.rotation_euler),
                "rotation_quaternion": list(pb.rotation_quaternion),
                "scale": list(pb.scale),
                "custom_props": {k: (list(v) if hasattr(v, "__len__") and not isinstance(v, str) else v)
                                 for k, v in b.items() if k in {
                                     "morpheus_id", "morpheus_name", "morpheus_xyz_mm",
                                     "morpheus_xyz_blender", "servo_id", "pwm_start",
                                     "pwm_low", "pwm_high", "pwm_direction"}},
                "constraints": [
                    {
                        "name": c.name, "type": c.type, "influence": c.influence, "mute": c.mute,
                        "target": c.target.name if hasattr(c, "target") and c.target else None,
                        "subtarget": c.subtarget if hasattr(c, "subtarget") else None,
                        "owner_space": c.owner_space if hasattr(c, "owner_space") else None,
                        "target_space": c.target_space if hasattr(c, "target_space") else None,
                        "use_x": c.use_x if hasattr(c, "use_x") else None,
                        "use_y": c.use_y if hasattr(c, "use_y") else None,
                        "use_z": c.use_z if hasattr(c, "use_z") else None,
                    }
                    for c in pb.constraints
                ],
            })
        report["armatures"][ob.name] = {"animation": anim_info(ob.data), "object_animation": anim_info(ob), "bones": bones}
for act in bpy.data.actions:
    report["actions"].append({
        "name": act.name,
        "frame_range": list(act.frame_range),
        "slots": [s.identifier for s in act.slots],
        "users": act.users,
    })
for text in bpy.data.texts:
    report["texts"][text.name] = text.as_string()
(ROOT / "rig_audit.json").write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
print("RIG_AUDIT", ROOT / "rig_audit.json")
print("FILE", bpy.data.filepath)
print("ARMATURES", {k: len(v["bones"]) for k, v in report["armatures"].items()})
print("ACTIONS", report["actions"])
print("TEXTS", list(report["texts"]))
