import sys, site
print("解释器:", sys.executable)
print("site-packages:", site.getsitepackages())

for p in site.getsitepackages():
    sys.path.insert(0, p)
sys.path.insert(0,"C:\\Users\\22805\\AppData\\Local\\Programs\\Python\\Python311\\Lib\\site-packages")
print("sys_path:", sys.path)
import bpy
import pandas as pd
print("pandas site:", pd.__file__)


def process_excel(file_path):
    df = pd.read_excel(file_path)
    required_columns = ['英文名称', '最小脉冲值', '最大脉冲值', '初始相对角度', '角度范围', '舵机ID']
    missing_cols = [col for col in required_columns if col not in df.columns]
    if missing_cols:
        raise ValueError(f"Excel文件中缺少必要的列: {missing_cols}")
    result = {}
    for index, row in df.iterrows():
        if pd.notna(row['英文名称']):
            data_list = [
                row['最小脉冲值'],
                row['最大脉冲值'],
                row['初始相对角度'],
                row['角度范围'],
                row['舵机ID']
            ]        
            result[row['英文名称']] = data_list
    return result

if __name__ == "__main__":
    excel_file = r"C:\Users\22805\Desktop\work\逸璟科技\elhead\BV03-02.xlsx"
    
    try:
        data_dict = process_excel(excel_file) 
        for bone_name, values in data_dict.items():
            print(f"正在处理{bone_name}: {values}")
            pose_bone = bpy.data.objects["Feace_Rig.001"].pose.bones[bone_name]
            servo_settings = pose_bone.bone.servo_settings

            target_min = int(values[0])
            target_max = int(values[1])
            
            if target_max > servo_settings.position_max:
                servo_settings.position_max = target_max
                servo_settings.position_min = target_min
            else:
                servo_settings.position_min = target_min
                servo_settings.position_max = target_max
            
            servo_settings.neutral_angle = float(values[2])
            servo_settings.rotation_range = float(values[3])
            servo_settings.servo_id = int(values[4])
            servo_settings.threshold = 2000
            
    except FileNotFoundError:
        print(f"错误: 找不到文件 {excel_file}")
    except ValueError as e:
        print(f"错误: {e}")
    except Exception as e:
        print(f"处理文件时发生错误: {e}")