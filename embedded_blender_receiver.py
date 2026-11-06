"""
Blender 端：接收 A2F UDP 数据，实时驱动 Shape Key

带帧缓冲 + 匀速回放，吸收网络抖动，避免卡顿

使用方法:
  1. 在 Blender 中打开 Scripting 工作区
  2. 打开或粘贴此脚本
  3. 修改下方 CONFIG 区域的参数
  4. 点击运行 — 会自动开始监听（无需额外点击）
  5. 在 Jetson 端运行 stream_to_blender.py
  6. 按 ESC 停止接收
"""

import bpy
import socket
import struct
import json
import time
from collections import deque

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIG — 根据你的环境修改
# ═══════════════════════════════════════════════════════════════════════════════

LISTEN_PORT = 9876          # 监听端口
MESH_NAME = "SampleHead"    # Mesh 对象名，空 = 自动查找
SK_PREFIX = ""              # Shape key 前缀
JITTER_BUFFER = 1           # 缓冲帧数（发送端已做预缓冲，局域网抖动极小，1帧即可）
TARGET_FPS = 30             # 目标回放帧率

# ═══════════════════════════════════════════════════════════════════════════════

MAGIC = b"A2F\x00"
TYPE_FRAME = 0x01
TYPE_STOP = 0x02
TYPE_META = 0x03
HEADER_FMT = "!4sB I d I"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

ARKIT_NAMES = [
    "eyeBlinkLeft", "eyeLookDownLeft", "eyeLookInLeft",
    "eyeLookOutLeft", "eyeLookUpLeft", "eyeSquintLeft",
    "eyeWideLeft", "eyeBlinkRight", "eyeLookDownRight",
    "eyeLookInRight", "eyeLookOutRight", "eyeLookUpRight",
    "eyeSquintRight", "eyeWideRight", "jawForward",
    "jawLeft", "jawRight", "jawOpen",
    "mouthClose", "mouthFunnel", "mouthPucker",
    "mouthLeft", "mouthRight", "mouthSmileLeft",
    "mouthSmileRight", "mouthFrownLeft", "mouthFrownRight",
    "mouthDimpleLeft", "mouthDimpleRight", "mouthStretchLeft",
    "mouthStretchRight", "mouthRollLower", "mouthRollUpper",
    "mouthShrugLower", "mouthShrugUpper", "mouthPressLeft",
    "mouthPressRight", "mouthLowerDownLeft", "mouthLowerDownRight",
    "mouthUpperUpLeft", "mouthUpperUpRight", "browDownLeft",
    "browDownRight", "browInnerUp", "browOuterUpLeft",
    "browOuterUpRight", "cheekPuff", "cheekSquintLeft",
    "cheekSquintRight", "noseSneerLeft", "noseSneerRight",
    "tongueOut",
]


def find_mesh(name_hint=""):
    if name_hint:
        obj = bpy.data.objects.get(name_hint)
        if obj and obj.type == 'MESH' and obj.data.shape_keys: 
            return obj
    for obj in bpy.data.objects:
        if obj.type == 'MESH' and obj.data.shape_keys:
            return obj
    return None


def build_key_map(mesh_obj, prefix=""):
    keys = mesh_obj.data.shape_keys.key_blocks
    key_dict = {k.name: k for k in keys}
    mapping = {}
    for i, name in enumerate(ARKIT_NAMES):
        for candidate in [prefix + name, name, name.lower()]:
            if candidate in key_dict:
                mapping[i] = key_dict[candidate]
                break
    return mapping


class A2F_OT_Receiver(bpy.types.Operator):
    bl_idname = "a2f.udp_receiver"
    bl_label = "A2F UDP Receiver"

    _timer = None
    _sock = None
    _mesh_obj = None
    _key_map = None
    _count = 0
    _log_interval = 60

    _frame_buf = None
    _playback_started = False
    _last_apply_pc = 0.0
    _frame_interval = 1.0 / TARGET_FPS
    _stream_ended = False

    def modal(self, context, event):
        if event.type == 'ESC':
            self._cleanup(context)
            print(f"[A2F] Stopped. Total frames applied: {self._count}")
            return {'CANCELLED'}

        if event.type != 'TIMER':
            return {'PASS_THROUGH'}

        # ── 1) 收包：读取所有待处理的 UDP 包，入缓冲 ──
        while True:
            try:
                data, addr = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except Exception as e:
                print(f"[A2F] recv error: {e}")
                break

            if len(data) < 5 or data[:4] != MAGIC:
                continue

            pkt_type = data[4]

            if pkt_type == TYPE_META:
                try:
                    meta = json.loads(data[5:])
                    fps = meta.get("fps", TARGET_FPS)
                    self._frame_interval = 1.0 / fps
                    print(f"[A2F] Meta: fps={fps}, interval={self._frame_interval*1000:.1f}ms")
                except:
                    pass
                continue

            if pkt_type == TYPE_STOP:
                self._stream_ended = True
                print(f"[A2F] Stream end signal received (buffered: {len(self._frame_buf)})")
                continue

            if pkt_type == TYPE_FRAME:
                if len(data) < HEADER_SIZE + 52 * 4:
                    continue
                _, _, idx, ts, count = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
                n = min(count, 52)
                weights = list(struct.unpack(f"!{n}f", data[HEADER_SIZE:HEADER_SIZE + n * 4]))
                self._frame_buf.append(weights)

        # ── 2) 匀速回放：每 1/60s 从缓冲中取一帧应用 ──
        if not self._playback_started:
            if len(self._frame_buf) >= JITTER_BUFFER:
                self._playback_started = True
                self._last_apply_pc = time.perf_counter()
                print(f"[A2F] Playback started (buffer: {len(self._frame_buf)})")

        applied = False
        if self._playback_started and self._frame_buf:
            now = time.perf_counter()
            elapsed = now - self._last_apply_pc

            frames_due = int(elapsed / self._frame_interval)
            if frames_due > 0:
                # 如果欠了多帧（Blender 卡了一下），跳到最新的该应用的帧
                skip = min(frames_due - 1, len(self._frame_buf) - 1)
                for _ in range(skip):
                    self._frame_buf.popleft()

                if self._frame_buf:
                    weights = self._frame_buf.popleft()
                    self._apply(weights)
                    self._count += 1
                    applied = True

                    if self._count <= 3 or self._count % self._log_interval == 0:
                        top3 = sorted(range(len(weights)), key=lambda i: -weights[i])[:3]
                        info = ", ".join(f"{ARKIT_NAMES[i]}={weights[i]:.3f}" for i in top3 if i < len(ARKIT_NAMES))
                        buf_sz = len(self._frame_buf)
                        print(f"[A2F] frame {self._count}: {info} (buf={buf_sz})")

                self._last_apply_pc += frames_due * self._frame_interval

        # 流结束且缓冲清空
        if self._stream_ended and not self._frame_buf and self._playback_started:
            print(f"[A2F] Playback finished. Total: {self._count} frames")
            self._stream_ended = False
            self._playback_started = False

        if applied:
            for area in context.screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()

        return {'RUNNING_MODAL'}

    def _apply(self, weights):
        if not self._key_map:
            return

        for i, kb in self._key_map.items():
            if i < len(weights):
                kb.value = max(0.0, min(1.0, weights[i]))

    def execute(self, context):
        mesh = find_mesh(MESH_NAME)
        if not mesh:
            self.report({'ERROR'}, "No mesh with shape keys found!")
            print("[A2F] ERROR: No mesh with shape keys. Set MESH_NAME in script.")
            return {'CANCELLED'}

        key_map = build_key_map(mesh, SK_PREFIX)
        if not key_map:
            self.report({'ERROR'}, f"No matching ARKit shape keys on '{mesh.name}'")
            print(f"[A2F] ERROR: No ARKit keys matched on '{mesh.name}'")
            return {'CANCELLED'}

        print(f"[A2F] Mesh: {mesh.name}")
        print(f"[A2F] Matched {len(key_map)}/52 shape keys")

        matched = [ARKIT_NAMES[i] for i in sorted(key_map.keys())]
        print(f"[A2F] Keys: {', '.join(matched[:10])}...")

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            sock.bind(("0.0.0.0", LISTEN_PORT))
            sock.setblocking(False)
        except OSError as e:
            print(f"[A2F] ERROR: Cannot bind port {LISTEN_PORT}: {e}")
            self.report({'ERROR'}, f"Port {LISTEN_PORT} in use")
            return {'CANCELLED'}

        self._sock = sock
        self._mesh_obj = mesh
        self._key_map = key_map
        self._count = 0
        self._frame_buf = deque()
        self._playback_started = False
        self._stream_ended = False
        self._last_apply_pc = 0.0

        self._timer = context.window_manager.event_timer_add(
            1.0 / 120, window=context.window
        )
        context.window_manager.modal_handler_add(self)

        print(f"[A2F] Listening on UDP :{LISTEN_PORT} — press ESC to stop")
        print(f"[A2F] Jitter buffer: {JITTER_BUFFER} frames, target {TARGET_FPS} FPS")
        self.report({'INFO'}, f"A2F listening on :{LISTEN_PORT}")
        return {'RUNNING_MODAL'}

    def _cleanup(self, context):
        if self._timer:
            context.window_manager.event_timer_remove(self._timer)
            self._timer = None
        if self._sock:
            self._sock.close()
            self._sock = None
        if self._key_map:
            for kb in self._key_map.values():
                kb.value = 0.0


# ═══════════════════════════════════════════════════════════════════════════════
# Panel
# ═══════════════════════════════════════════════════════════════════════════════

class A2F_PT_Panel(bpy.types.Panel):
    bl_label = "Audio2Face"
    bl_idname = "A2F_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "A2F"

    def draw(self, context):
        layout = self.layout
        mesh = find_mesh(MESH_NAME)
        if mesh:
            km = build_key_map(mesh, SK_PREFIX)
            layout.label(text=f"Mesh: {mesh.name} ({len(km)}/52 keys)")
        else:
            layout.label(text="No mesh with shape keys", icon='ERROR')
        layout.label(text=f"Port: {LISTEN_PORT}")
        layout.operator("a2f.udp_receiver", text="Start Receiving", icon='PLAY')


# ═══════════════════════════════════════════════════════════════════════════════
# Register & Auto-start
# ═══════════════════════════════════════════════════════════════════════════════

classes = (A2F_OT_Receiver, A2F_PT_Panel)

def register():
    for cls in classes:
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass
        bpy.utils.register_class(cls)

def unregister():
    for cls in reversed(classes):
        try:
            bpy.utils.unregister_class(cls)
        except:
            pass

if __name__ == "__main__":
    register()
    print()
    print("=" * 50)
    print("  A2F Receiver — Auto Starting...")
    print("=" * 50)
    bpy.ops.a2f.udp_receiver('INVOKE_DEFAULT')
