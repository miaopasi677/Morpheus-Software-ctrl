import bpy
import csv
import os

# CSV文件路径 - 修改为你的CSV文件路径
#csv_file_path = r"C:/Users/12979/Desktop/中东/v1/camera_blendshape_data_1751365554.csv"
#csv_file_path = r"C:/Users/12979/Desktop/102仿生头验收/102表情同步/2号头/表情录制/厌恶.csv"
#csv_file_path = r"C:/Users/12979/Desktop/102仿生头验收/102表情同步/camera_blendshape_data_1758541819.csv"
# csv_file_path = r"C:/Users/12979/Desktop/逸璟灵智/3D数字人/test_arkit_insert/test_arkit_mouth.csv"
# csv_file_path = r"D:\\Jiaming\\Ezren\\damiao\\docs\\viseme_v2.csv"
#自然1 悲伤0 鄙视1 调皮0 烦躁0 高兴0 害怕0 惊喜0 惊吓0 惊讶0 撒娇0 生气0
csv_file_path = r"D:\project\elhead\expression\explib\arkit\自然.csv"
# 形态键名称映射
#name_mapping = {
#    'EyeBlinkLeft': 'eyeBlinkLeft',
#    'EyeLookDownLeft': 'eyeLookDownLeft',
#    'EyeLookInLeft': 'eyeLookInLeft',
#    'EyeLookOutLeft': 'eyeLookOutLeft',
#    'EyeLookUpLeft': 'eyeLookUpLeft',
#    'EyeSquintLeft': 'eyeSquintLeft',
#    'EyeWideLeft': 'eyeWideLeft',
#    'EyeBlinkRight': 'eyeBlinkRight',
#    'EyeLookDownRight': 'eyeLookDownRight',
#    'EyeLookInRight': 'eyeLookInRight',
#    'EyeLookOutRight': 'eyeLookOutRight',
#    'EyeLookUpRight': 'eyeLookUpRight',
#    'EyeSquintRight': 'eyeSquintRight',
#    'EyeWideRight': 'eyeWideRight',
#    'JawForward': 'jawForward',
#    'JawRight': 'jawRight',
#    'JawLeft': 'jawLeft',
#    'JawOpen': 'jawOpen',
#    'MouthClose': 'mouthClose',
#    'MouthFunnel': 'mouthFunnel',
#    'MouthPucker': 'mouthPucker',
#    'MouthRight': 'mouthRight',
#    'MouthLeft': 'mouthLeft',
#    'MouthSmileLeft': 'mouthSmileLeft',
#    'MouthSmileRight': 'mouthSmileRight',
#    'MouthFrownLeft': 'mouthFrownLeft',
#    'MouthFrownRight': 'mouthFrownRight',
#    'MouthDimpleLeft': 'mouthDimpleLeft',
#    'MouthDimpleRight': 'mouthDimpleRight',
#    'MouthStretchLeft': 'mouthStretchLeft',
#    'MouthStretchRight': 'mouthStretchRight',
#    'MouthRollLower': 'mouthRollLower',
#    'MouthRollUpper': 'mouthRollUpper',
#    'MouthShrugLower': 'mouthShrugLower',
#    'MouthShrugUpper': 'mouthShrugUpper',
#    'MouthPressLeft': 'mouthPressLeft',
#    'MouthPressRight': 'mouthPressRight',
#    'MouthLowerDownLeft': 'mouthLowerDownLeft',
#    'MouthLowerDownRight': 'mouthLowerDownRight',
#    'MouthUpperUpLeft': 'mouthUpperUpLeft',
#    'MouthUpperUpRight': 'mouthUpperUpRight',
#    'BrowDownLeft': 'browDownLeft',
#    'BrowDownRight': 'browDownRight',
#    'BrowInnerUp': 'browInnerUp',
#    'BrowOuterUpLeft': 'browOuterUpLeft',
#    'BrowOuterUpRight': 'browOuterUpRight',
#    'CheekPuff': 'cheekPuff',
#    'CheekSquintLeft': 'cheekSquintLeft',
#    'CheekSquintRight': 'cheekSquintRight',
#    'NoseSneerLeft': 'noseSneerLeft',
#    'NoseSneerRight': 'noseSneerRight',
#    'TongueOut': 'tongueOut',
#    'HeadYaw': 'headYaw',
#    'HeadPitch': 'headPitch',
#    'HeadRoll': 'headRoll',
#    'LeftEyeYaw': 'leftEyeYaw',
#    'LeftEyePitch': 'leftEyePitch',
#    'LeftEyeRoll': 'leftEyeRoll',
#    'RightEyeYaw': 'rightEyeYaw',
#    'RightEyePitch': 'rightEyePitch',
#    'RightEyeRoll': 'rightEyeRoll'
#}
name_mapping = {
    'eyeBlinkLeft': 'eyeBlinkLeft',
    'eyeLookDownLeft': 'eyeLookDownLeft',
    'eyeLookInLeft': 'eyeLookInLeft',
    'eyeLookOutLeft': 'eyeLookOutLeft',
    'eyeLookUpLeft': 'eyeLookUpLeft',
    'eyeSquintLeft': 'eyeSquintLeft',
    'eyeWideLeft': 'eyeWideLeft',
    'eyeBlinkRight': 'eyeBlinkRight',
    'eyeLookDownRight': 'eyeLookDownRight',
    'eyeLookInRight': 'eyeLookInRight',
    'eyeLookOutRight': 'eyeLookOutRight',
    'eyeLookUpRight': 'eyeLookUpRight',
    'eyeSquintRight': 'eyeSquintRight',
    'eyeWideRight': 'eyeWideRight',
    'jawForward': 'jawForward',
    'jawRight': 'jawRight',
    'jawLeft': 'jawLeft',
    'jawOpen': 'jawOpen',
    'mouthClose': 'mouthClose',
    'mouthFunnel': 'mouthFunnel',
    'mouthPucker': 'mouthPucker',
    'mouthRight': 'mouthRight',
    'mouthLeft': 'mouthLeft',
    'mouthSmileLeft': 'mouthSmileLeft',
    'mouthSmileRight': 'mouthSmileRight',
    'mouthFrownLeft': 'mouthFrownLeft',
    'mouthFrownRight': 'mouthFrownRight',
    'mouthDimpleLeft': 'mouthDimpleLeft',
    'mouthDimpleRight': 'mouthDimpleRight',
    'mouthStretchLeft': 'mouthStretchLeft',
    'mouthStretchRight': 'mouthStretchRight',
    'mouthRollLower': 'mouthRollLower',
    'mouthRollUpper': 'mouthRollUpper',
    'mouthShrugLower': 'mouthShrugLower',
    'mouthShrugUpper': 'mouthShrugUpper',
    'mouthPressLeft': 'mouthPressLeft',
    'mouthPressRight': 'mouthPressRight',
    'mouthLowerDownLeft': 'mouthLowerDownLeft',
    'mouthLowerDownRight': 'mouthLowerDownRight',
    'mouthUpperUpLeft': 'mouthUpperUpLeft',
    'mouthUpperUpRight': 'mouthUpperUpRight',
    'browDownLeft': 'browDownLeft',
    'browDownRight': 'browDownRight',
    'browInnerUp': 'browInnerUp',
    'browOuterUpLeft': 'browOuterUpLeft',
    'browOuterUpRight': 'browOuterUpRight',
    'cheekPuff': 'cheekPuff',
    'cheekSquintLeft': 'cheekSquintLeft',
    'cheekSquintRight': 'cheekSquintRight',
    'noseSneerLeft': 'noseSneerLeft',
    'noseSneerRight': 'noseSneerRight',
    'tongueOut': 'tongueOut',
    'headYaw': 'headYaw',
    'headPitch': 'headPitch',
    'headRoll': 'headRoll',
    'leftEyeYaw': 'leftEyeYaw',
    'leftEyePitch': 'leftEyePitch',
    'leftEyeRoll': 'leftEyeRoll',
    'rightEyeYaw': 'rightEyeYaw',
    'rightEyePitch': 'rightEyePitch',
    'rightEyeRoll': 'rightEyeRoll'
}

def get_armature_object():
    """获取场景中的骨架对象"""
    for obj in bpy.context.scene.objects:
        if obj.type == 'ARMATURE':
            return obj
    return None

def clear_bone_animation(armature, bone_name):
    """清除骨骼的动画数据"""
    if armature.animation_data and armature.animation_data.action:
        # 移除所有与该骨骼相关的动画曲线
        fcurves_to_remove = []
        for fcurve in armature.animation_data.action.fcurves:
            if fcurve.data_path.startswith(f'pose.bones["{bone_name}"]'):
                fcurves_to_remove.append(fcurve)
        
        for fcurve in fcurves_to_remove:
            armature.animation_data.action.fcurves.remove(fcurve)

# 在你的主代码中使用
head_bone = bpy.data.objects["Facemotion_TESTAVATAR"].pose.bones["head"]
armature = bpy.data.objects["Facemotion_TESTAVATAR"]  # 获取骨架对象
clear_bone_animation(armature, "head")  # 清除头部骨骼的动画数据

def print_debug_info():
    """打印调试信息"""
    print("=== 调试信息 ===")
    
    # 检查当前选择的对象
    obj = bpy.context.active_object
    if obj:
        print(f"当前选择的对象: {obj.name}")
        print(f"对象类型: {obj.type}")
        
        if obj.type == 'MESH':
            print("✓ 对象是网格类型")
            if obj.data.shape_keys:
                shape_keys = obj.data.shape_keys.key_blocks
                print(f"✓ 对象有形态键，共 {len(shape_keys)} 个")
                print("现有形态键列表:")
                for i, key in enumerate(shape_keys):
                    print(f"  {i}: {key.name}")
            else:
                print("✗ 对象没有形态键")
        else:
            print(f"✗ 对象不是网格类型，是 {obj.type}")
    else:
        print("✗ 没有选择任何对象")
    
    # 检查骨架
    armature = get_armature_object()
    if armature:
        print(f"\n找到骨架: {armature.name}")
        if 'Skull' in armature.pose.bones:
            print("✓ 骨架包含 'Skull' 骨骼")
        else:
            print("✗ 骨架不包含 'Skull' 骨骼")
    else:
        print("✗ 场景中没有找到骨架对象")
    
    print("=== 调试信息结束 ===\n")

def check_existing_shapekeys(obj):
    """检查对象现有的形态键"""
    if not obj.data.shape_keys:
        return set(), set()
    
    existing_keys = set(key.name for key in obj.data.shape_keys.key_blocks)
    needed_keys = set(name_mapping.values())
    available_keys = needed_keys.intersection(existing_keys)
    missing_keys = needed_keys - existing_keys
    
    return available_keys, missing_keys

def main():
    # 打印调试信息
    print_debug_info()
    
    # 选择当前活动的模型对象
    obj = bpy.context.active_object
    if not obj:
        raise Exception("请先选择一个对象")
    
    # 检查对象类型
    if obj.type != 'MESH':
        raise Exception(f"所选对象类型是 {obj.type}，需要选择网格(MESH)对象")
    
    # 检查是否有形态键
    if not obj.data.shape_keys:
        raise Exception("所选对象没有形态键。请手动为对象添加形态键后再运行此脚本。")
    
    print("✓ 对象已有形态键")
    
    # 检查现有形态键
    available_keys, missing_keys = check_existing_shapekeys(obj)
    
    if available_keys:
        print(f"✓ 找到 {len(available_keys)} 个可用的形态键")
    
    if missing_keys:
        print(f"⚠ 缺少 {len(missing_keys)} 个形态键:")
        for key_name in sorted(missing_keys):
            print(f"    - {key_name}")
        print("  这些形态键将被跳过。")
    
    # 读取CSV文件数据
    animation_data = []
    try:
        with open(csv_file_path, 'r', encoding='utf-8') as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                animation_data.append(row)
    except Exception as e:
        print(f"读取CSV文件时出错: {str(e)}")
        print("尝试使用不同的编码...")
        try:
            with open(csv_file_path, 'r', encoding='latin-1') as file:
                csv_reader = csv.DictReader(file)
                for row in csv_reader:
                    animation_data.append(row)
            print("使用latin-1编码成功读取CSV")
        except Exception as e2:
            raise Exception(f"无法读取CSV文件: {str(e2)}")
    
    if not animation_data:
        raise Exception("CSV文件为空或格式不正确")
    
    print(f"成功读取CSV文件，共 {len(animation_data)} 行数据")
    
    # 获取帧速率和持续时间
    scene = bpy.context.scene
    fps = scene.render.fps / scene.render.fps_base
    print(fps)
    frame_count = len(animation_data)
    
    # 设置场景帧范围
    scene.frame_start = 1
    scene.frame_end = frame_count
    
    # 清除现有的动画数据
    if obj.data.shape_keys.animation_data:
        obj.data.shape_keys.animation_data_clear()
    
    # 映射并创建关键帧
    shape_keys = obj.data.shape_keys.key_blocks
    valid_blendshapes = []
    missing_blendshapes = []
    
    # 检查哪些形态键存在于模型中
    for csv_name, blender_name in name_mapping.items():
        if blender_name in shape_keys:
            valid_blendshapes.append((csv_name, blender_name))
        else:
            missing_blendshapes.append((csv_name, blender_name))
    
    # 打印缺失的形态键
    if missing_blendshapes:
        print("警告：以下形态键在模型中不存在：")
        for csv_name, blender_name in missing_blendshapes:
            print(f"  CSV: '{csv_name}' -> Blender: '{blender_name}'")
    
    print(f"开始导入动画数据...")
    
    # 获取骨架和Skull骨骼
#    armature = get_armature_object()
#    skull_bone = None
#    if armature and 'Skull' in armature.pose.bones:
#        skull_bone = armature.pose.bones['Skull']
        # 清除骨骼现有的动画数据
#        clear_bone_animation(armature, 'Skull')
    head_bone = bpy.data.objects["Facemotion_TESTAVATAR"].pose.bones["head"]
    armature = bpy.data.objects["Facemotion_TESTAVATAR"]  # 获取骨架对象
    clear_bone_animation(armature, "head")  # 清除头部骨骼的动画数据
    
    # 为每个有效的形态键创建关键帧
    for frame_idx, frame_data in enumerate(animation_data):
        current_frame = frame_idx + 1
        scene.frame_set(current_frame)
        
        if frame_idx % 100 == 0:  # 每100帧显示进度
            print(f"  处理第 {current_frame} 帧...")
        
        # 处理形态键动画
        for csv_name, blender_name in valid_blendshapes:
            if csv_name in frame_data:
                try:
                    value = float(frame_data[csv_name])
                    # 确保值在0-1范围内
                    value = max(0.0, min(1.0, value))
                    shape_keys[blender_name].value = value
                    shape_keys[blender_name].keyframe_insert("value", frame=current_frame)
                except (ValueError, KeyError) as e:
                    print(f"处理 {csv_name} -> {blender_name} 时出错: {str(e)}")
        
        # 处理head骨骼旋转（脖子数据）
        if head_bone:
            try:
                # 获取脖子数据（已经是弧度）
                head_yaw = -float(frame_data.get('headYaw', 0))  # X轴取反
                head_pitch = float(frame_data.get('headPitch', 0))
                head_roll = float(frame_data.get('headRoll', 0))
                new_rotation = (head_yaw, head_pitch, head_roll)
                
                # 设置骨骼旋转
                head_bone.rotation_euler = new_rotation
                
                # 插入关键帧
                head_bone.keyframe_insert(data_path="rotation_euler", frame=current_frame)
                
            except (ValueError, KeyError) as e:
                print(f"处理head骨骼旋转时出错: {str(e)}")
    
    print(f"✓ 已完成导入 {len(valid_blendshapes)} 个形态键的动画数据，共 {frame_count} 帧")
    if head_bone:
        print("✓ 已成功将脖子数据绑定到head骨骼")
    else:
        print("⚠ 未找到head骨骼，脖子数据未导入")
    
    if missing_blendshapes:
        print(f"⚠ 缺失 {len(missing_blendshapes)} 个形态键")
    
    # 回到第一帧
    scene.frame_set(1)
    print("导入完成！")

# 运行主函数
if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"错误: {str(e)}")
        print("\n解决建议:")
        print("1. 确保选择了一个网格(Mesh)对象")
        print("2. 确保对象已经有形态键（在属性面板 > 网格属性 > 形态键中添加）")
        print("3. 检查CSV文件路径是否正确") 
        print("4. 确保CSV文件格式正确")
        print("5. 检查形态键名称是否与脚本中的映射匹配")
        print("6. 确保场景中有包含Skull骨骼的骨架")