import bpy
import math
import mathutils
import json
import os

# 骨骼列表
bones_list = [
    "Brow_Out_Rotation.r",
    "Brow_Out_Rotation.l", 
    "Brow_In_Rotation_pitch.r",
    "Brow_In_Rotation_pitch.l",
    
    "Eyelid_Up.r",
    "Eyelid_Up.l",
    "Eyelid_Dwn.r",
    "Eyelid_Dwn.l",
    
    "Eye_yaw.r",
    "Eye_yaw.l",
    "Eye_pitch.r",
    "Eye_pitch.l",

    "Lip_Up_Rotation.r",
    "Lip_Up_Rotation_C",
    "Lip_Up_Out_C",
    "Lip_Up_Rotation.l",
    "Lip_Dwn_Rotation.r",
    "Lip_Dwn_Rotation_C",
    "Lip_Dwn_Out_C",
    "Lip_Dwn_Rotation.l",
    
    "Smile_Rotation.r",
    "Lip_Corner_Rotation.r",
    "Smile_Rotation.l",
    "Lip_Corner_Rotation.l",
    
    "Jaw_Open",
    "Jaw_Yaw",
    
    "Neck_1",
    "Neck_2",
    "Neck_3"
]

#bones_configure = {
#    "Brow_Out_Rotation.r": [1250, 1900, "True"],
#    "Brow_Out_Rotation.l": [1350, 1950, "False"],
#    "Brow_In_Rotation_pitch.r": [1450, 1750, "True"],
#    "Brow_In_Rotation_pitch.l": [1450, 1750, "True"],
#    "Brow_In_Rotation_yaw.r": [1550, 1850, "True"],
#    "Brow_In_Rotation_yaw.l": [1100, 1400, "True"],
#    
#    "Eye_yaw.r": [1100, 1900, "True"],
#    "Eye_yaw.l": [1100, 2000, "False"],
#    "Eye_pitch.r": [1300, 2050, "True"],
#    "Eye_pitch.l": [900, 1600, "True"],
#    
#    "Eyelid_Up.r": [850, 1480, "True"],
#    "Eyelid_Up.l": [1300, 2000, "False"],

#    "Lip_Up_Rotation.r": [1550, 1900, "False"],
#    "Lip_Up_Rotation_C": [1450, 1850, "False"],
#    "Lip_Up_Out_C": [1400, 1900, "False"],
#    "Lip_Up_Rotation.l": [1350, 1750, "True"],
#    "Lip_Dwn_Rotation.r": [1100, 1700, "False"],
#    "Lip_Dwn_Rotation_C": [1450, 1800, "True"],
#    "Lip_Dwn_Out_C": [1300, 1800, "False"],
#    "Lip_Dwn_Rotation.l": [1300, 1900, "True"],
#    
#    "Smile_Rotation.r": [1200, 1350, "True"],
#    "Lip_Corner_Rotation.r": [1300, 1800, "False"],
#    "Smile_Rotation.l": [1320, 1470, "False"],
#    "Lip_Corner_Rotation.l": [1200, 1800, "True"],
#    
#    "Jaw_Open": [1550, 2000, "False"],
#    "Jaw_Yaw": [1400, 1750, "True"],
#    
#    "Neck.r": [500, 2000, "True"],
#    "Neck_C": [500, 2000, "False"],
#    "Neck.l": [500, 2000, "False"]
#}


#def pos2reg(bone, pos):
#    min_pos, max_pos, flag_inverse = bones_configure[bone]
#    reg_pos = (pos - min_pos) / (max_pos - min_pos)
#    if reg_pos < 0:
#        reg_pos = 0
#    elif reg_pos > 1:
#        reg_pos = 1

#    if flag_inverse == "True":
#        reg_pos = 1 - reg_pos

#    return reg_pos
    

def matrix_visual(pose_bone):
    bone = pose_bone.bone
    matrix_pose_bone = pose_bone.matrix
    matrix_bone = bone.matrix_local
    if bone.parent:
        matrix_parent_bone = bone.parent.matrix_local.copy()
        matrix_parent_pose_bone = pose_bone.parent.matrix.copy()
    else:
        matrix_parent_bone = mathutils.Matrix()
        matrix_parent_pose_bone = mathutils.Matrix()
    matrix_bone_inverted = matrix_bone.copy().inverted()
    matrix_parent_pose_bone_inverted = matrix_parent_pose_bone.copy().inverted()
    return (
        matrix_bone_inverted
        @ matrix_parent_bone
        @ matrix_parent_pose_bone_inverted
        @ matrix_pose_bone
    )

def range_map(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low



def extract_frame(frame_number):
    """提取指定帧的骨骼数据"""
    # 设置当前帧
    bpy.context.scene.frame_set(frame_number)
    frame_data = []
      
    for bone_name in bones_list:
        try:
            pose_bone = bpy.data.objects["Feace_Rig.001"].pose.bones[bone_name]
            servo_settings = pose_bone.bone.servo_settings
            rotation_euler = matrix_visual(pose_bone).to_euler()
            rotation_axis_index = int(servo_settings.rotation_axis)
            rotation_in_degrees = round(math.degrees(
                rotation_euler[rotation_axis_index]) * servo_settings.multiplier, 2)
            
            if servo_settings.reverse_direction:
                rotation_in_degrees = rotation_in_degrees * -1
            
            angle = servo_settings.neutral_angle - rotation_in_degrees
            position = round(range_map(angle, 0, servo_settings.rotation_range,
                              servo_settings.position_min, servo_settings.position_max))
            in_range = servo_settings.position_min <= position <= servo_settings.position_max
            
            
            frame_data.append(position)
            
        except Exception as e:
            print(f"Error processing bone {bone_name} at frame {frame_number}: {e}")

    return frame_data

def extract_all_frames():
    """提取所有帧的数据"""
    scene = bpy.context.scene
    start_frame = scene.frame_start
    end_frame = scene.frame_end
    
    all_frames_data = []
    
    print(f"开始提取第 {start_frame} 到第 {end_frame} 帧的数据...")
    
    for frame in range(start_frame, end_frame + 1): 
        print(f"正在处理第 {frame} 帧...")
        frame_data = extract_frame(frame)
        all_frames_data.append(frame_data)
          
    return all_frames_data

def save_to_file(data, expression, filepath):
    expression_data = {
    expression: data
}
    
    """保存数据到JSON文件"""
    # 获取Blender文件所在目录
#    blend_file_path = bpy.data.filepath
#    if blend_file_path:
#        output_dir = os.path.dirname(blend_file_path)
#    else:
#        output_dir = os.path.expanduser("~")  # 如果没有保存.blend文件，则保存到用户目录
#    
#    output_path = os.path.join(output_dir, filepath)
    
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            # 方法1：使用自定义格式化
            f.write('{\n')
            f.write(f'  "{expression}": [\n')
            
            for i, row in enumerate(data):
                # 每个子数组写在一行
                row_json = json.dumps(row, ensure_ascii=False, separators=(',', ':'))
                if i < len(data) - 1:
                    f.write(f'    {row_json},\n')
                else:
                    f.write(f'    {row_json}\n')
            
            f.write('  ]\n')
            f.write('}\n')
        
        print(f"数据已保存到: {output_path}")
    except Exception as e:
        print(f"保存文件时出错: {e}")

def print_frame_data(frame_number):
    """打印指定帧的详细数据（用于调试）"""
    frame_data = extract_frame_data(frame_number)
    print(f"\n第 {frame_number} 帧详细数据:")
    print("-" * 50)
    for bone_name, bone_data in frame_data["bones"].items():
        if "error" in bone_data:
            print(f"{bone_name}: ERROR - {bone_data['error']}")
        else:
            print(f"{bone_name}: position={bone_data['position']}, angle={bone_data['angle']}, in_range={bone_data['in_range']}")

if __name__ == "__main__":
    # 方式1：提取所有帧的数据并保存到JSON文件
    all_data = extract_all_frames()
    headname = "LinZhi05"
    expression = "mouth1"
    filebase = "C:\\Users\\12979\\Desktop\\逸璟灵智\\EZRobot\\Expression\\explib\\robots"
    filepath = os.path.join(filebase, headname, expression+".json")

    
    print(all_data)
    save_to_file(all_data, expression, filepath)
    
    # 方式2：只提取当前帧的数据（用于测试）
    # current_frame = bpy.context.scene.frame_current
    # print_frame_data(current_frame)
    
    # 方式3：提取指定帧范围的数据
    # start = 1
    # end = 10
    # selected_frames_data = []
    # for frame in range(start, end + 1):
    #     frame_data = extract_frame_data(frame)
    #     selected_frames_data.append(frame_data)
    # save_to_file(selected_frames_data, f"frames_{start}_to_{end}.json")
    
    print("数据提取完成！")