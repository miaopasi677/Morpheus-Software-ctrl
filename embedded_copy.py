import bpy
import zipfile
import xml.etree.ElementTree as ET

NS = {'s': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}

def read_xlsx(file_path, sheet='xl/worksheets/sheet1.xml'):
    """用标准库读取 xlsx 第一个工作表，返回 list[dict]，key 是表头名"""
    with zipfile.ZipFile(file_path) as z:
        # 1. 读取共享字符串表（xlsx 把字符串单独存一份，单元格里只存索引）
        shared = []
        if 'xl/sharedStrings.xml' in z.namelist():
            root = ET.parse(z.open('xl/sharedStrings.xml')).getroot()
            for si in root.findall('s:si', NS):
                # 一个 si 里可能有多个 t 片段，拼起来
                shared.append(''.join(t.text or '' for t in si.iter(f'{{{NS["s"]}}}t')))

        # 2. 读取工作表
        root = ET.parse(z.open(sheet)).getroot()
        rows = []
        for row in root.iter(f'{{{NS["s"]}}}row'):
            cells = {}
            for c in row.findall('s:c', NS):
                ref = c.get('r')                      # 比如 "B3"
                col = ''.join(ch for ch in ref if ch.isalpha())
                t = c.get('t', 'n')
                v = c.find('s:v', NS)
                if v is None or v.text is None:
                    continue
                val = v.text
                if t == 's':                           # 共享字符串
                    val = shared[int(val)]
                elif t in ('n', 'str', 'inlineStr'):
                    try:
                        f = float(val)
                        val = int(f) if f.is_integer() else f
                    except ValueError:
                        pass
                cells[col] = val
            if cells:
                rows.append(cells)
        return rows


def process_excel(file_path):
    rows = read_xlsx(file_path)
    if not rows:
        raise ValueError("Excel 文件为空")

    # 第一行作表头：列字母 -> 表头名
    header = {col: str(v) for col, v in rows[0].items()}
    # 反转：表头名 -> 列字母
    name2col = {v: k for k, v in header.items()}

    required = ['骨骼名称', '最小脉冲值', '最大脉冲值', '初始相对角度', '角度范围', '舵机ID']
    missing = [c for c in required if c not in name2col]
    if missing:
        raise ValueError(f"Excel文件中缺少必要的列: {missing}")

    result = {}
    for row in rows[1:]:
        name_cell = row.get(name2col['骨骼名称'])
        if name_cell is None or str(name_cell).strip() == '':
            continue
        result[str(name_cell)] = [
            row.get(name2col['最小脉冲值']),
            row.get(name2col['最大脉冲值']),
            row.get(name2col['初始相对角度']),
            row.get(name2col['角度范围']),
            row.get(name2col['舵机ID']),
        ]
    return result


if __name__ == "__main__":
    excel_file = r"C:\Users\zbw64\Downloads\ttt.xlsx"

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