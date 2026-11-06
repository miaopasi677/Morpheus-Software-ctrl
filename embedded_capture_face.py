import bpy
import socket
import threading

# 形态键名称映射（与采集端保持一致）
name_mapping = [
    'EyeBlinkLeft', 'EyeLookDownLeft', 'EyeLookInLeft', 'EyeLookOutLeft', 'EyeLookUpLeft',
    'EyeSquintLeft', 'EyeWideLeft', 'EyeBlinkRight', 'EyeLookDownRight', 'EyeLookInRight',
    'EyeLookOutRight', 'EyeLookUpRight', 'EyeSquintRight', 'EyeWideRight', 'JawForward',
    'JawRight', 'JawLeft', 'JawOpen', 'MouthClose', 'MouthFunnel', 'MouthPucker', 'MouthRight',
    'MouthLeft', 'MouthSmileLeft', 'MouthSmileRight', 'MouthFrownLeft', 'MouthFrownRight',
    'MouthDimpleLeft', 'MouthDimpleRight', 'MouthStretchLeft', 'MouthStretchRight',
    'MouthRollLower', 'MouthRollUpper', 'MouthShrugLower', 'MouthShrugUpper', 'MouthPressLeft',
    'MouthPressRight', 'MouthLowerDownLeft', 'MouthLowerDownRight', 'MouthUpperUpLeft',
    'MouthUpperUpRight', 'BrowDownLeft', 'BrowDownRight', 'BrowInnerUp', 'BrowOuterUpLeft',
    'BrowOuterUpRight', 'CheekPuff', 'CheekSquintLeft', 'CheekSquintRight', 'NoseSneerLeft',
    'NoseSneerRight', 'TongueOut', 'HeadYaw', 'HeadPitch', 'HeadRoll', 'LeftEyeYaw',
    'LeftEyePitch', 'LeftEyeRoll', 'RightEyeYaw', 'RightEyePitch', 'RightEyeRoll'
]

blender_mapping = {
    'EyeBlinkLeft': 'eyeBlinkLeft',
    'EyeLookDownLeft': 'eyeLookDownLeft',
    'EyeLookInLeft': 'eyeLookInLeft',
    'EyeLookOutLeft': 'eyeLookOutLeft',
    'EyeLookUpLeft': 'eyeLookUpLeft',
    'EyeSquintLeft': 'eyeSquintLeft',
    'EyeWideLeft': 'eyeWideLeft',
    'EyeBlinkRight': 'eyeBlinkRight',
    'EyeLookDownRight': 'eyeLookDownRight',
    'EyeLookInRight': 'eyeLookInRight',
    'EyeLookOutRight': 'eyeLookOutRight',
    'EyeLookUpRight': 'eyeLookUpRight',
    'EyeSquintRight': 'eyeSquintRight',
    'EyeWideRight': 'eyeWideRight',
    'JawForward': 'jawForward',
    'JawRight': 'jawRight',
    'JawLeft': 'jawLeft',
    'JawOpen': 'jawOpen',
    'MouthClose': 'mouthClose',
    'MouthFunnel': 'mouthFunnel',
    'MouthPucker': 'mouthPucker',
    'MouthRight': 'mouthRight',
    'MouthLeft': 'mouthLeft',
    'MouthSmileLeft': 'mouthSmileLeft',
    'MouthSmileRight': 'mouthSmileRight',
    'MouthFrownLeft': 'mouthFrownLeft',
    'MouthFrownRight': 'mouthFrownRight',
    'MouthDimpleLeft': 'mouthDimpleLeft',
    'MouthDimpleRight': 'mouthDimpleRight',
    'MouthStretchLeft': 'mouthStretchLeft',
    'MouthStretchRight': 'mouthStretchRight',
    'MouthRollLower': 'mouthRollLower',
    'MouthRollUpper': 'mouthRollUpper',
    'MouthShrugLower': 'mouthShrugLower',
    'MouthShrugUpper': 'mouthShrugUpper',
    'MouthPressLeft': 'mouthPressLeft',
    'MouthPressRight': 'mouthPressRight',
    'MouthLowerDownLeft': 'mouthLowerDownLeft',
    'MouthLowerDownRight': 'mouthLowerDownRight',
    'MouthUpperUpLeft': 'mouthUpperUpLeft',
    'MouthUpperUpRight': 'mouthUpperUpRight',
    'BrowDownLeft': 'browDownLeft',
    'BrowDownRight': 'browDownRight',
    'BrowInnerUp': 'browInnerUp',
    'BrowOuterUpLeft': 'browOuterUpLeft',
    'BrowOuterUpRight': 'browOuterUpRight',
    'CheekPuff': 'cheekPuff',
    'CheekSquintLeft': 'cheekSquintLeft',
    'CheekSquintRight': 'cheekSquintRight',
    'NoseSneerLeft': 'noseSneerLeft',
    'NoseSneerRight': 'noseSneerRight',
    'TongueOut': 'tongueOut',
    'HeadYaw': 'headYaw',
    'HeadPitch': 'headPitch',
    'HeadRoll': 'headRoll',
    'LeftEyeYaw': 'leftEyeYaw',
    'LeftEyePitch': 'leftEyePitch',
    'LeftEyeRoll': 'leftEyeRoll',
    'RightEyeYaw': 'rightEyeYaw',
    'RightEyePitch': 'rightEyePitch',
    'RightEyeRoll': 'rightEyeRoll'
}

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# 请根据实际情况修改下面两个变量
FACE_SHAPEKEYS_NAME = "Key"      # 脸部 shape_keys 名称
EYE_SHAPEKEYS_NAME = "Key.001"   # 眼睛 shape_keys 名称

def apply_blendshape_data(data_list):
    # 脸部
    try:
        face_keys = bpy.data.shape_keys[FACE_SHAPEKEYS_NAME].key_blocks
        for i, key in enumerate(name_mapping):
            if key in blender_mapping and blender_mapping[key] in face_keys and i < len(data_list) - 9:
                try:
                    value = float(data_list[i])
                    value = max(0.0, min(1.0, value))
                    face_keys[blender_mapping[key]].value = value
                except Exception:
                    pass
    except Exception as e:
        print("脸部赋值失败：", e)

    # 眼睛
    try:
        eye_keys = bpy.data.shape_keys[EYE_SHAPEKEYS_NAME].key_blocks
        # 眼睛相关的8个shape key，索引需与发送端顺序一致
        for i, key in enumerate(name_mapping):
            if key in blender_mapping and blender_mapping[key] in face_keys and i < len(data_list) - 9:
                try:
                    value = float(data_list[i])
                    value = max(0.0, min(1.0, value))
                    eye_keys[blender_mapping[key]].value = value
                except Exception:
                    pass
    except Exception as e:
        print("眼睛赋值失败：", e)

    # 头部骨骼
    try:
        armature = bpy.data.objects["Facemotion_TESTAVATAR"]
        head_bone = armature.pose.bones["head"]
        head_yaw = -float(data_list[-9])
        head_pitch = float(data_list[-8])
        head_roll = float(data_list[-7])
        head_bone.rotation_mode = 'XYZ'
        head_bone.rotation_euler = (head_yaw, head_pitch, head_roll)
    except Exception as e:
        print("头部骨骼赋值失败：", e)

    bpy.context.view_layer.update()

def listen_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Blender listening on {UDP_IP}:{UDP_PORT}")
    while True:
        data, addr = sock.recvfrom(8192)
        data_str = data.decode('utf-8')
        data_list = data_str.strip().split(',')
        bpy.app.timers.register(lambda: apply_blendshape_data(data_list), first_interval=0)

# 启动监听线程
thread = threading.Thread(target=listen_socket, daemon=True)
thread.start()
print("Blender实时接收线程已启动")