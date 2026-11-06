import bpy
import json
from pathlib import Path

arm = bpy.data.objects["Feace_Rig.001"]
mesh = bpy.data.objects["SampleHead"]
keys = mesh.data.shape_keys
print("ACTION", keys.animation_data.action.name if keys.animation_data and keys.animation_data.action else None)
print("SLOT", keys.animation_data.action_slot.identifier if keys.animation_data and keys.animation_data.action_slot else None)
print("FRAME_RANGE", bpy.context.scene.frame_start, bpy.context.scene.frame_end)
for name in ("browInnerE.R", "jawE", "lipConnerE.R"):
    ob = bpy.data.objects[name]
    print("PARENT", name, ob.parent_type, list(ob.parent_vertices), list(ob.matrix_world.translation))
for frame in (1, 5, 10, 15, 20, 25, 30):
    bpy.context.scene.frame_set(frame)
    bpy.context.view_layer.update()
    depsgraph = bpy.context.evaluated_depsgraph_get()
    top = sorted(((k.name, k.value) for k in keys.key_blocks), key=lambda item: -item[1])[:8]
    print("FRAME", frame, "TOP_KEYS", top)
    for name in ("browInnerE.R", "jawE", "lipConnerE.R"):
        ob = bpy.data.objects[name]
        ev = ob.evaluated_get(depsgraph)
        print("TARGET", name, "orig", list(ob.matrix_world.translation), "eval", list(ev.matrix_world.translation))
    for name in ("Brow_In_Rotation_pitch.r", "Jaw_Open", "Smile_Rotation.r"):
        pb = arm.pose.bones[name]
        print("BONE", name, list(pb.rotation_euler), list(pb.matrix.translation))
    evaluated_mesh = mesh.evaluated_get(depsgraph)
    for name in ("browInnerE.R", "jawE", "lipConnerE.R"):
        ob = bpy.data.objects[name]
        vi = ob.parent_vertices[0]
        print("VERTEX", name, vi, list(evaluated_mesh.data.vertices[vi].co))
print("DRIVERS")
for fc in arm.animation_data.drivers:
    print(fc.data_path, fc.driver.is_valid, fc.driver.expression)
