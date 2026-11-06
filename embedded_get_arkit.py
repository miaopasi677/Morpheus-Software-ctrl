import bpy
import csv
import os

def export_facemotion_to_csv(output_path, mesh_name="SampleHead", armature_name="Facemotion_TESTAVATAR", bone_name="head"):
    """
    导出面部动画数据到CSV文件
    
    参数:
        output_path: CSV文件保存路径
        mesh_name: 包含形态键的网格体名称
        armature_name: 骨架名称
        bone_name: 头部骨骼名称
    """
    
    # 获取网格体对象
    mesh_obj = bpy.data.objects.get(mesh_name)
    if not mesh_obj or not mesh_obj.data.shape_keys:
        print(f"错误: 找不到网格体 '{mesh_name}' 或其没有形态键")
        return
    
    # 获取骨架对象
    armature_obj = bpy.data.objects.get(armature_name)
    if not armature_obj:
        print(f"错误: 找不到骨架 '{armature_name}'")
        return
    
    # 获取骨骼
    pose_bone = armature_obj.pose.bones.get(bone_name)
    if not pose_bone:
        print(f"错误: 找不到骨骼 '{bone_name}'")
        return
    
    # 获取形态键（跳过Basis）
    shape_keys = mesh_obj.data.shape_keys.key_blocks
    blendshape_names = [sk.name for sk in shape_keys if sk.name != "Basis"]
    
    # 构建CSV表头
    headers = ["frame"] + blendshape_names + ["headPitch", "headRoll", "headYaw"]
    
    # 获取动画帧范围
    scene = bpy.context.scene
    frame_start = scene.frame_start
    frame_end = scene.frame_end
    
    # 收集数据
    rows = []
    for frame in range(frame_start, frame_end + 1):
        scene.frame_set(frame)
        
        row = [frame]
        
        # 获取形态键值
        for sk_name in blendshape_names:
            value = shape_keys[sk_name].value
            row.append(value)
        
        # 获取骨骼旋转（欧拉角）
        rotation = pose_bone.rotation_euler
        row.append(rotation.x)  
        row.append(rotation.y) 
        row.append(rotation.z) 
        
        rows.append(row)
    
    # 写入CSV文件
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)
    
    print(f"成功导出 {len(rows)} 帧数据到: {output_path}")
    print(f"形态键数量: {len(blendshape_names)}")

# ============ 使用示例 ============

if __name__ == "__main__":
    # 设置输出路径
    #自然1 悲伤0 鄙视1 调皮0 烦躁0 高兴0 害怕0 惊喜0 惊吓0 惊讶0 撒娇0 生气0
    output_file = r"C:\Users\12979\Desktop\biaoqing\撒娇.csv"  # 修改为你想保存的路径

    # 执行导出（根据你的实际对象名称修改参数）
    export_facemotion_to_csv(
        output_path=output_file,
        mesh_name="SampleHead",           # 你的网格体名称
        armature_name="Facemotion_TESTAVATAR",  # 你的骨架名称
        bone_name="head"                  # 头部骨骼名称
    )
    # bpy.data.scenes["Scene"].frame_end = bpy.data.actions["mouth1"].frame_end
