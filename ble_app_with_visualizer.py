import asyncio
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
import json

import csv
import os
from datetime import datetime

from bleak import BleakClient, BleakScanner

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ===== NUS UUIDs (Nordic UART Service) =====
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # Central -> Peripheral (write)
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # Peripheral -> Central (notify)


class BleWorker:
    """
    Chạy Bleak + asyncio loop trong 1 thread riêng để không block Tkinter.
    GUI gọi worker bằng cách đưa command vào queue.
    Worker đẩy log/status ngược lại GUI qua queue.
    """
    def __init__(self, ui_queue: queue.Queue):
        self.ui_q = ui_queue

        self.cmd_q = queue.Queue()
        self.thread = None
        self.loop = None

        self.client: BleakClient | None = None
        self.connected_addr: str | None = None

        self.stop_flag = threading.Event()
        self.want_connected = False   # GUI set True khi muốn giữ kết nối
        self.auto_reconnect = True

        self.device_name = "BACKBELT"
        self.device_address = ""      # nếu set thì ưu tiên connect theo address

        # ===== CSV logging =====
        self.csv_enabled = True
        self.csv_path = "logs/nus_log.csv"
        self._csv_fp = None
        self._csv_writer = None
        self._csv_lock = threading.Lock()

        # Data rate tracking
        self.data_count = 0
        self.last_data_time = time.time()

    # ---------- UI helpers ----------
    def _ui_log(self, msg: str):
        self.ui_q.put(("log", msg))

    def _ui_status(self, msg: str):
        self.ui_q.put(("status", msg))

    def _ui_conn(self, is_connected: bool):
        self.ui_q.put(("conn", is_connected))

    def _ui_imu_data(self, yaw: float, pitch: float, roll: float):
        self.data_count += 1
        # Chỉ gửi nếu queue chưa đầy, tránh lag khi nhận 100Hz
        try:
            self.ui_q.put_nowait(("imu", (yaw, pitch, roll)))
        except queue.Full:
            # Queue đầy, bỏ qua packet này (UI sẽ dùng data mới hơn)
            pass

    def _ui_mag_data(self, mag_x: float, mag_y: float, mag_z: float):
        # Gửi dữ liệu magnetometer về UI
        try:
            self.ui_q.put_nowait(("mag", (mag_x, mag_y, mag_z)))
        except queue.Full:
            pass

    # ---------- CSV helpers ----------
    def set_csv(self, enabled: bool, path: str):
        self.csv_enabled = bool(enabled)
        if path:
            self.csv_path = path

    def _csv_open_if_needed(self):
        if not self.csv_enabled:
            return
        if self._csv_fp:
            return

        folder = os.path.dirname(self.csv_path)
        if folder:
            os.makedirs(folder, exist_ok=True)

        is_new = not os.path.exists(self.csv_path)

        # Buffered write (8KB) để giảm I/O
        self._csv_fp = open(self.csv_path, "a", newline="", encoding="utf-8", buffering=8192)
        self._csv_writer = csv.writer(self._csv_fp)

        if is_new:
            # Header
            self._csv_writer.writerow(["ts_iso", "text"])
            self._csv_fp.flush()

    def _csv_close(self):
        with self._csv_lock:
            try:
                if self._csv_fp:
                    self._csv_fp.flush()
                    self._csv_fp.close()
            finally:
                self._csv_fp = None
                self._csv_writer = None

    # ---------- Public API from GUI ----------
    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_flag.clear()
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()

    def shutdown(self):
        # dừng thread + loop
        self.stop_flag.set()
        self.want_connected = False
        self.cmd_q.put(("disconnect", None))
        self.cmd_q.put(("stop", None))

    def set_target(self, name: str, address: str):
        self.device_name = (name or "").strip()
        self.device_address = (address or "").strip()

    def set_auto_reconnect(self, enabled: bool):
        self.auto_reconnect = enabled

    def connect(self):
        self.want_connected = True
        self.cmd_q.put(("connect", None))

    def disconnect(self):
        self.want_connected = False
        self.cmd_q.put(("disconnect", None))

    def reconnect(self):
        # reconnect = disconnect rồi connect lại
        self.want_connected = True
        self.cmd_q.put(("reconnect", None))

    def send(self, data: bytes, with_response: bool = False):
        self.cmd_q.put(("send", (data, with_response)))

    def scan_devices(self, timeout: float = 8.0):
        """Yêu cầu scan các thiết bị BLE và trả kết quả về UI."""
        self.cmd_q.put(("scan", timeout))

    # ---------- Thread/async loop ----------
    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self._ui_status("BLE worker started.")
        self.loop.create_task(self._runner())

        try:
            self.loop.run_forever()
        finally:
            # cleanup
            try:
                pending = asyncio.all_tasks(self.loop)
                for t in pending:
                    t.cancel()
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self.loop.close()

    async def _runner(self):
        """
        Vòng lặp chính: xử lý command từ GUI + auto-reconnect.
        """
        while not self.stop_flag.is_set():
            # xử lý command (non-blocking)
            try:
                cmd, payload = self.cmd_q.get_nowait()
            except queue.Empty:
                cmd = None
                payload = None

            if cmd == "stop":
                self._ui_status("Stopping BLE loop...")
                await self._safe_disconnect()
                self._ui_conn(False)
                self.loop.call_soon_threadsafe(self.loop.stop)
                return

            if cmd == "connect":
                await self._connect_flow()

            elif cmd == "disconnect":
                await self._safe_disconnect()

            elif cmd == "reconnect":
                await self._safe_disconnect()
                await asyncio.sleep(0.2)
                await self._connect_flow()

            elif cmd == "send":
                data, with_resp = payload
                await self._send_flow(data, with_resp)

            elif cmd == "scan":
                timeout = payload
                await self._scan_flow(timeout)

            # auto reconnect nếu đang "muốn kết nối" nhưng bị rớt
            if self.auto_reconnect and self.want_connected:
                if not (self.client and self.client.is_connected):
                    # tránh spam reconnect quá nhanh
                    await asyncio.sleep(0.5)
                    await self._connect_flow()

            await asyncio.sleep(0.05)

    async def _scan_flow(self, timeout: float = 8.0):
        """Scan tất cả thiết bị BLE và gửi danh sách về UI."""
        self._ui_status(f"Scanning BLE devices ({timeout:.0f}s)...")
        self._ui_log(f"Scanning for BLE devices ({timeout:.0f}s)...")
        try:
            devices = await BleakScanner.discover(timeout=timeout)
            device_list = []
            for d in devices:
                name = d.name if d.name else "(Unknown)"
                device_list.append((name, d.address))
                self._ui_log(f"Found: {name} @ {d.address}")
            
            self.ui_q.put(("devices", device_list))
            self._ui_status(f"Scan complete. Found {len(device_list)} devices.")
            self._ui_log(f"✅ Scan complete. Found {len(device_list)} devices.")
        except Exception as e:
            self._ui_log(f"❌ Scan error: {e}")
            self._ui_status("Scan error.")

    async def _find_by_name(self, name: str, timeout: float = 8.0) -> str | None:
        self._ui_log(f"Scanning for name='{name}' ({timeout:.0f}s)...")
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name and name.upper() in d.name.upper():
                self._ui_log(f"Found: {d.name} @ {d.address}")
                return d.address
        return None

    def _on_nus_notify(self, sender: int, data: bytearray):
        raw = bytes(data)
        hexs = raw.hex()
        txt = ""

        try:
            txt = raw.decode("utf-8", errors="strict").strip()
            self._ui_log(f"[RX] {txt}")
            
            # Parse IMU data
            self._parse_imu_data(txt)
            
        except Exception as e:
            self._ui_log(f"Parse error: {e}")

        # ===== Save to CSV (buffered, flush mỗi 50 dòng) =====
        if self.csv_enabled and txt:
            with self._csv_lock:
                try:
                    self._csv_open_if_needed()
                    ts_iso = datetime.now().isoformat(timespec="milliseconds")
                    self._csv_writer.writerow([ts_iso, txt])
                    # Flush mỗi 50 dòng thay vì mỗi lần để tăng tốc
                    if self.data_count % 50 == 0:
                        self._csv_fp.flush()
                except Exception as e:
                    self._ui_log(f"❌ CSV write error: {e}")

    def _parse_imu_data(self, txt: str):
        """
        Parse IMU data from various formats:
        - JSON: {"sensor":{"time":78016,"status":true,"accel":[...],"gyro":[...],"ypr":[-12.27,-55.82,11.58],"mag":[...]}}
        - JSON: {"time":31221,"accel":[0.02,0.24,-8.11],"gyro":[0.00,0.00,0.00],"ypr":[-31.54,-2.23,0.05],"mag":[...]}
        - CSV: "yaw,pitch,roll"
        - Colon: "Y:yaw P:pitch R:roll"
        - Space: "yaw pitch roll"
        """
        try:
            # Try JSON format first
            if txt.strip().startswith('{'):
                data = json.loads(txt)
                
                # Check for nested sensor format: {"sensor":{"ypr":[...],"mag":[...]}}
                if 'sensor' in data and isinstance(data['sensor'], dict):
                    sensor = data['sensor']
                    if 'ypr' in sensor and isinstance(sensor['ypr'], list) and len(sensor['ypr']) >= 3:
                        roll = float(sensor['ypr'][0])
                        pitch = float(sensor['ypr'][1])
                        yaw = float(sensor['ypr'][2])
                        self._ui_imu_data(yaw, pitch, roll)
                    
                    # Parse magnetometer data if available
                    if 'mag' in sensor and isinstance(sensor['mag'], list) and len(sensor['mag']) >= 3:
                        mag_x = float(sensor['mag'][0])
                        mag_y = float(sensor['mag'][1])
                        mag_z = float(sensor['mag'][2])
                        self._ui_mag_data(mag_x, mag_y, mag_z)
                    return
                
                # Check for direct ypr format: {"ypr":[...],"mag":[...]}
                if 'ypr' in data and isinstance(data['ypr'], list) and len(data['ypr']) >= 3:
                    roll = float(data['ypr'][0])
                    pitch = float(data['ypr'][1])
                    yaw = float(data['ypr'][2])
                    self._ui_imu_data(yaw, pitch, roll)
                
                # Parse magnetometer data if available
                if 'mag' in data and isinstance(data['mag'], list) and len(data['mag']) >= 3:
                    mag_x = float(data['mag'][0])
                    mag_y = float(data['mag'][1])
                    mag_z = float(data['mag'][2])
                    self._ui_mag_data(mag_x, mag_y, mag_z)
                    return
                
                if 'ypr' in data:
                    return
            
            # Try comma-separated format
            if ',' in txt:
                parts = txt.split(',')
                if len(parts) >= 3:
                    yaw = float(parts[0].strip())
                    pitch = float(parts[1].strip())
                    roll = float(parts[2].strip())
                    self._ui_imu_data(yaw, pitch, roll)
                    return
            
            # Try colon format "Y:yaw P:pitch R:roll"
            if 'Y:' in txt or 'y:' in txt:
                txt_lower = txt.lower()
                y_idx = txt_lower.find('y:')
                p_idx = txt_lower.find('p:')
                r_idx = txt_lower.find('r:')
                
                if y_idx >= 0 and p_idx >= 0 and r_idx >= 0:
                    yaw_str = txt[y_idx+2:p_idx].strip()
                    pitch_str = txt[p_idx+2:r_idx].strip()
                    roll_str = txt[r_idx+2:].strip()
                    
                    yaw = float(yaw_str)
                    pitch = float(pitch_str)
                    roll = float(roll_str)
                    self._ui_imu_data(yaw, pitch, roll)
                    return
            
            # Try space-separated format
            parts = txt.split()
            if len(parts) >= 3:
                yaw = float(parts[0])
                pitch = float(parts[1])
                roll = float(parts[2])
                self._ui_imu_data(yaw, pitch, roll)
                return
                
        except Exception:
            # Không parse được thì bỏ qua
            pass

    def _on_disconnect_cb(self, client: BleakClient):
        # callback của bleak chạy ngoài asyncio context => chỉ log
        self._ui_log("⚠️ Disconnected from device.")
        self._ui_conn(False)
        self._ui_status("Disconnected.")

    async def _connect_flow(self):
        if self.client and self.client.is_connected:
            self._ui_status("Already connected.")
            self._ui_conn(True)
            return

        # resolve address
        addr = self.device_address if self.device_address else None
        if not addr:
            if not self.device_name:
                self._ui_log("❌ Please input device name or address.")
                self._ui_status("No target.")
                self._ui_conn(False)
                return
            addr = await self._find_by_name(self.device_name, timeout=8.0)

        if not addr:
            self._ui_log("❌ Device not found. Check name/address and advertising.")
            self._ui_status("Not found.")
            self._ui_conn(False)
            return

        self._ui_status(f"Connecting to {addr} ...")
        self._ui_log(f"Connecting to: {addr}")

        try:
            self.client = BleakClient(addr, disconnected_callback=self._on_disconnect_cb)
            await self.client.connect(timeout=10.0)
            self.connected_addr = addr

            if not self.client.is_connected:
                raise RuntimeError("Connect failed.")

            # ensure services discovered (Bleak thường auto-discover sau connect)
            try:
                await self.client.get_services()
            except Exception:
                pass

            await self.client.start_notify(NUS_TX_UUID, self._on_nus_notify)
            self._ui_log("✅ Subscribed to NUS TX notify.")
            self._ui_status("Connected.")
            self._ui_conn(True)

        except Exception as e:
            self._ui_log(f"❌ Connect error: {e}")
            self._ui_status("Connect error.")
            self._ui_conn(False)
            await self._safe_disconnect()

    async def _safe_disconnect(self):
        if not self.client:
            self._ui_status("Disconnected.")
            self._ui_conn(False)
            # cũng đóng CSV (nếu đang mở)
            self._csv_close()
            return
        try:
            if self.client.is_connected:
                try:
                    await self.client.stop_notify(NUS_TX_UUID)
                except Exception:
                    pass
                await self.client.disconnect()
        except Exception as e:
            self._ui_log(f"(disconnect error ignored) {e}")
        finally:
            self.client = None
            self.connected_addr = None
            self._ui_status("Disconnected.")
            self._ui_conn(False)
            self._csv_close()

    async def _send_flow(self, data: bytes, with_response: bool):
        if not (self.client and self.client.is_connected):
            self._ui_log("❌ Not connected. Cannot send.")
            return
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=with_response)
            self._ui_log(f"[TX] Sent {len(data)} bytes: {data!r}")
        except Exception as e:
            self._ui_log(f"❌ Send error: {e}")


class MagVisualizer:
    """
    Visualize Magnetometer data in real-time line plot
    """
    def __init__(self, parent_frame):
        self.fig = Figure(figsize=(6, 2.5), dpi=60)
        self.ax = self.fig.add_subplot(111)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Data buffers (keep last 100 points)
        self.max_points = 100
        self.mag_x_data = []
        self.mag_y_data = []
        self.mag_z_data = []
        self.time_data = []
        self.start_time = time.time()
        
        # Line objects
        self.line_x, = self.ax.plot([], [], 'r-', linewidth=1.5, label='Mag X')
        self.line_y, = self.ax.plot([], [], 'g-', linewidth=1.5, label='Mag Y')
        self.line_z, = self.ax.plot([], [], 'b-', linewidth=1.5, label='Mag Z')
        
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Magnetometer (μT)')
        self.ax.set_title('Magnetometer Real-Time Data')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, alpha=0.3)
        
        # Frame skipping
        self.skip_counter = 0
        self.skip_every = 2  # Vẽ mỗi 3 frame
        
    def update(self, mag_x, mag_y, mag_z):
        """Update magnetometer plot with new data"""
        # Frame skipping
        self.skip_counter += 1
        if self.skip_counter <= self.skip_every:
            return
        self.skip_counter = 0
        
        current_time = time.time() - self.start_time
        
        # Add new data
        self.time_data.append(current_time)
        self.mag_x_data.append(mag_x)
        self.mag_y_data.append(mag_y)
        self.mag_z_data.append(mag_z)
        
        # Keep only last max_points
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.mag_x_data.pop(0)
            self.mag_y_data.pop(0)
            self.mag_z_data.pop(0)
        
        # Update lines
        self.line_x.set_data(self.time_data, self.mag_x_data)
        self.line_y.set_data(self.time_data, self.mag_y_data)
        self.line_z.set_data(self.time_data, self.mag_z_data)
        
        # Auto-scale axes
        if len(self.time_data) > 0:
            self.ax.set_xlim(max(0, current_time - 10), current_time + 0.5)  # Show last 10 seconds
            
            all_data = self.mag_x_data + self.mag_y_data + self.mag_z_data
            if all_data:
                y_min = min(all_data)
                y_max = max(all_data)
                margin = (y_max - y_min) * 0.1 if y_max != y_min else 1
                self.ax.set_ylim(y_min - margin, y_max + margin)
        
        self.canvas.draw_idle()
        self.canvas.flush_events()


class IMUVisualizer:
    """
    Visualize IMU orientation in 3D using matplotlib - OPTIMIZED VERSION
    """
    def __init__(self, parent_frame):
        # DPI tối ưu cho cân bằng chất lượng/hiệu suất
        self.fig = Figure(figsize=(6, 3.5), dpi=60)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Current orientation
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        # Cache previous orientation để skip render khi không đổi
        self.prev_yaw = None
        self.prev_pitch = None
        self.prev_roll = None
        
        # Frame skipping tối thiểu - vẽ nhiều frame hơn
        self.skip_counter = 0
        self.skip_every = 0  # Vẽ MỌI frame (không skip)
        
        # Cache cho legend
        self.legend_drawn = False
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        # Cache cho 3D objects để tái sử dụng
        self.box_collection = None
        self.axes_lines = []
        
        self._setup_plot()
        self._draw_object()

    def _setup_plot(self):
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('IMU Orientation Visualization')

    def _rotation_matrix(self, yaw, pitch, roll):
        """
        Create rotation matrix from Euler angles (in degrees)
        """
        yaw_rad = np.radians(yaw)
        pitch_rad = np.radians(pitch)
        roll_rad = np.radians(roll)
        
        # Rotation around Z axis (yaw)
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Rotation around Y axis (pitch)
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # Rotation around X axis (roll)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        return R

    def _draw_object(self):
        """
        Draw 3D box and axes arrows - OPTIMIZED with full 6-face box
        """
        self.ax.clear()
        self._setup_plot()
        
        # Box vertices
        vertices = np.array([
            [-0.7, -0.35, -0.12],  # 0: bottom-left-back
            [0.7, -0.35, -0.12],   # 1: bottom-right-back
            [0.7, 0.35, -0.12],    # 2: bottom-right-front
            [-0.7, 0.35, -0.12],   # 3: bottom-left-front
            [-0.7, -0.35, 0.12],   # 4: top-left-back
            [0.7, -0.35, 0.12],    # 5: top-right-back
            [0.7, 0.35, 0.12],     # 6: top-right-front
            [-0.7, 0.35, 0.12]     # 7: top-left-front
        ])
        
        # Apply rotation
        R = self._rotation_matrix(self.yaw, self.pitch, self.roll)
        rotated_vertices = vertices @ R.T
        
        # VẼ TẤT CẢ 6 mặt của box để hiển thị rõ hơn
        faces = [
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[2], rotated_vertices[3]],  # bottom
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],  # top
            [rotated_vertices[0], rotated_vertices[1], rotated_vertices[5], rotated_vertices[4]],  # back
            [rotated_vertices[3], rotated_vertices[2], rotated_vertices[6], rotated_vertices[7]],  # front
            [rotated_vertices[0], rotated_vertices[3], rotated_vertices[7], rotated_vertices[4]],  # left
            [rotated_vertices[1], rotated_vertices[2], rotated_vertices[6], rotated_vertices[5]]   # right
        ]
        
        # Box với edges mỏng để dễ nhìn
        poly3d = Poly3DCollection(faces, facecolors=['cyan', 'yellow', 'lightblue', 'lightgreen', 'pink', 'orange'], 
                                 alpha=0.3, edgecolors='black', linewidths=0.5)
        self.ax.add_collection3d(poly3d)
        
        # Draw axes arrows với độ dày rõ hơn
        axis_length = 1.2
        axes = np.array([
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length]
        ])
        rotated_axes = axes @ R.T
        
        # Arrows dày hơn
        self.ax.plot([0, rotated_axes[0, 0]], [0, rotated_axes[0, 1]], [0, rotated_axes[0, 2]], 
                     'r-', linewidth=2.5)
        self.ax.plot([0, rotated_axes[1, 0]], [0, rotated_axes[1, 1]], [0, rotated_axes[1, 2]], 
                     'g-', linewidth=2.5)
        self.ax.plot([0, rotated_axes[2, 0]], [0, rotated_axes[2, 1]], [0, rotated_axes[2, 2]], 
                     'b-', linewidth=2.5)
        
        # Legend vẽ 1 lần
        if not self.legend_drawn:
            self.ax.plot([], [], 'r-', linewidth=2.5, label='X (Forward)')
            self.ax.plot([], [], 'g-', linewidth=2.5, label='Y (Left)')
            self.ax.plot([], [], 'b-', linewidth=2.5, label='Z (Up)')
            self.ax.legend(loc='upper right', fontsize=9)
            self.legend_drawn = True
        
        # Draw với blit=True để nhanh hơn (chỉ vẽ phần thay đổi)
        self.canvas.draw_idle()
        self.canvas.flush_events()

    def update(self, yaw, pitch, roll):
        """
        Update orientation - OPTIMIZED: Skip nếu giá trị không đổi
        """
        # Skip nếu giá trị không thay đổi (tiết kiệm CPU)
        if (self.prev_yaw == yaw and self.prev_pitch == pitch and self.prev_roll == roll):
            return
        
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        
        self.prev_yaw = yaw
        self.prev_pitch = pitch
        self.prev_roll = roll
        
        # Frame skipping (0 = vẽ mọi frame)
        self.skip_counter += 1
        if self.skip_counter <= self.skip_every:
            return
        
        self.skip_counter = 0
        
        # Vẽ frame
        self._draw_object()
        
        # Track FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:
            fps = self.frame_count / (current_time - self.last_fps_time)
            print(f"[3D Render FPS: {fps:.1f}]")
            self.frame_count = 0
            self.last_fps_time = current_time


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("BLE NUS Application with IMU Visualizer + CSV Logging")
        self.geometry("1400x800")

        self.ui_q = queue.Queue(maxsize=200)
        self.worker = BleWorker(self.ui_q)

        # Set CSV file name by timestamp
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.worker.set_csv(True, f"logs/nus_{ts}.csv")

        self.worker.start()
        
        # Tracking data rate
        self.data_rate_count = 0
        self.data_rate_time = time.time()
        self.current_data_rate = 0.0

        # Dictionary để lưu thông tin thiết bị
        self.devices_dict = {}

        self._build_ui()
        self.after(1, self._poll_ui_queue)  # ULTRA FAST: Poll mỗi 1ms

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        # Main container
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # Left panel: controls + log
        left_panel = ttk.Frame(main, width=700)
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 10))

        # Right panel: 3D visualization
        right_panel = ttk.Frame(main, width=600)
        right_panel.pack(side="right", fill="both", expand=True)

        # ===== LEFT PANEL =====
        # --- Connection controls ---
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding=10)
        conn_frame.pack(fill="x")

        ttk.Label(conn_frame, text="Device name:").grid(row=0, column=0, sticky="w")
        self.var_name = tk.StringVar(value="BACKBELT")
        ttk.Entry(conn_frame, textvariable=self.var_name, width=20).grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(conn_frame, text="Address:").grid(row=0, column=2, sticky="w", padx=(10, 0))
        self.var_addr = tk.StringVar(value="")
        ttk.Entry(conn_frame, textvariable=self.var_addr, width=25).grid(row=0, column=3, sticky="w", padx=5)

        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn_frame, text="Auto reconnect", variable=self.var_auto,
                       command=self._apply_settings).grid(row=0, column=4, sticky="w", padx=(10, 0))

        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=1, column=0, columnspan=5, pady=(10, 0))

        self.btn_scan    = ttk.Button(btn_frame, text="Scan", command=self._scan_devices)
        self.btn_connect = ttk.Button(btn_frame, text="Connect", command=self._connect)
        self.btn_disc    = ttk.Button(btn_frame, text="Disconnect", command=self._disconnect)
        self.btn_reconn  = ttk.Button(btn_frame, text="Reconnect", command=self._reconnect)

        self.btn_scan.pack(side="left", padx=2)
        self.btn_connect.pack(side="left", padx=2)
        self.btn_disc.pack(side="left", padx=2)
        self.btn_reconn.pack(side="left", padx=2)

        # Status
        status_frame = ttk.Frame(conn_frame)
        status_frame.grid(row=2, column=0, columnspan=5, pady=(5, 0))

        ttk.Label(status_frame, text="Status:").pack(side="left")
        self.var_status = tk.StringVar(value="Idle")
        ttk.Label(status_frame, textvariable=self.var_status, foreground="blue").pack(side="left", padx=5)

        self.var_conn = tk.StringVar(value="DISCONNECTED")
        self.lbl_conn = ttk.Label(status_frame, textvariable=self.var_conn, foreground="red")
        self.lbl_conn.pack(side="left", padx=10)

        # --- Discovered Devices ---
        devf = ttk.LabelFrame(left_panel, text="Discovered Devices", padding=10)
        devf.pack(fill="x", pady=(10, 0))

        list_frame = ttk.Frame(devf)
        list_frame.pack(fill="both", expand=True)

        self.device_listbox = tk.Listbox(list_frame, height=4)
        self.device_listbox.pack(side="left", fill="both", expand=True)
        self.device_listbox.bind("<<ListboxSelect>>", self._on_device_select)

        dev_sb = ttk.Scrollbar(list_frame, command=self.device_listbox.yview)
        dev_sb.pack(side="right", fill="y")
        self.device_listbox.configure(yscrollcommand=dev_sb.set)

        # --- Send command ---
        cmd_frame = ttk.LabelFrame(left_panel, text="Send Command", padding=10)
        cmd_frame.pack(fill="x", pady=(10, 0))

        ttk.Label(cmd_frame, text="Command:").grid(row=0, column=0, sticky="w")
        self.var_cmd = tk.StringVar(value="hello\\n")
        ttk.Entry(cmd_frame, textvariable=self.var_cmd, width=40).grid(
            row=0, column=1, sticky="we", padx=5)

        self.var_resp = tk.BooleanVar(value=False)
        ttk.Checkbutton(cmd_frame, text="With response", variable=self.var_resp).grid(
            row=0, column=2, sticky="w", padx=5)

        ttk.Button(cmd_frame, text="Send", command=self._send).grid(
            row=0, column=3, sticky="w", padx=5)

        # Auto-send controls
        ttk.Label(cmd_frame, text="Interval (ms):").grid(row=1, column=0, sticky="w", pady=(5, 0))
        self.var_interval = tk.StringVar(value="1000")
        ttk.Entry(cmd_frame, textvariable=self.var_interval, width=10).grid(
            row=1, column=1, sticky="w", padx=5, pady=(5, 0))

        self.auto_send_active = False
        self.auto_send_job = None
        self.btn_auto_send = ttk.Button(cmd_frame, text="▶ Auto Send", command=self._toggle_auto_send)
        self.btn_auto_send.grid(row=1, column=2, columnspan=2, sticky="w", padx=5, pady=(5, 0))

        cmd_frame.grid_columnconfigure(1, weight=1)

        # --- IMU data display ---
        imu_frame = ttk.LabelFrame(left_panel, text="IMU Data", padding=10)
        imu_frame.pack(fill="x", pady=(10, 0))

        ttk.Label(imu_frame, text="Yaw:").grid(row=0, column=0, sticky="w", pady=3)
        self.var_yaw = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_yaw, width=12, font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=1, sticky="w", pady=3)

        ttk.Label(imu_frame, text="Pitch:").grid(row=0, column=2, sticky="w", pady=3, padx=(20, 0))
        self.var_pitch = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_pitch, width=12, font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=3, sticky="w", pady=3)

        ttk.Label(imu_frame, text="Roll:").grid(row=0, column=4, sticky="w", pady=3, padx=(20, 0))
        self.var_roll = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_roll, width=12, font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=5, sticky="w", pady=3)
        
        ttk.Label(imu_frame, text="Data Rate:").grid(row=1, column=0, sticky="w", pady=3)
        self.var_data_rate = tk.StringVar(value="0 Hz")
        ttk.Label(imu_frame, textvariable=self.var_data_rate, width=12, foreground="blue").grid(
            row=1, column=1, sticky="w", pady=3)
        
        # Magnetometer data
        ttk.Label(imu_frame, text="Mag X:").grid(row=2, column=0, sticky="w", pady=3)
        self.var_mag_x = tk.StringVar(value="0.0 μT")
        ttk.Label(imu_frame, textvariable=self.var_mag_x, width=12, font=("TkDefaultFont", 9)).grid(
            row=2, column=1, sticky="w", pady=3)
        
        ttk.Label(imu_frame, text="Mag Y:").grid(row=2, column=2, sticky="w", pady=3, padx=(20, 0))
        self.var_mag_y = tk.StringVar(value="0.0 μT")
        ttk.Label(imu_frame, textvariable=self.var_mag_y, width=12, font=("TkDefaultFont", 9)).grid(
            row=2, column=3, sticky="w", pady=3)
        
        ttk.Label(imu_frame, text="Mag Z:").grid(row=2, column=4, sticky="w", pady=3, padx=(20, 0))
        self.var_mag_z = tk.StringVar(value="0.0 μT")
        ttk.Label(imu_frame, textvariable=self.var_mag_z, width=12, font=("TkDefaultFont", 9)).grid(
            row=2, column=5, sticky="w", pady=3)

        # --- Log area ---
        log_frame = ttk.LabelFrame(left_panel, text="Log", padding=10)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.txt = tk.Text(log_frame, wrap="word", height=18)
        self.txt.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(log_frame, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)

        # ===== RIGHT PANEL =====
        # --- 3D Visualization ---
        viz_frame = ttk.LabelFrame(right_panel, text="3D IMU Orientation", padding=10)
        viz_frame.pack(fill="both", expand=True)

        self.visualizer = IMUVisualizer(viz_frame)
        
        # --- Magnetometer Visualization ---
        mag_frame = ttk.LabelFrame(right_panel, text="Magnetometer Data", padding=10)
        mag_frame.pack(fill="both", expand=False, pady=(10, 0))
        
        self.mag_visualizer = MagVisualizer(mag_frame)

        # hint
        self._log("🚀 OPTIMIZED MODE: DPI 60, 0-skip rendering, 1ms poll, buffered CSV")
        self._log("Ready. Connect to device for real-time IMU + Mag visualization.")
        self._log(f"CSV saving to: {self.worker.csv_path}")
        self._log("Expected data: JSON {'ypr':[y,p,r], 'mag':[x,y,z]} or 'yaw,pitch,roll'")
        self._set_connected(False)

    def _apply_settings(self):
        self.worker.set_auto_reconnect(self.var_auto.get())

    def _update_target(self):
        self.worker.set_target(self.var_name.get(), self.var_addr.get())
        self.worker.set_auto_reconnect(self.var_auto.get())

    def _connect(self):
        self._update_target()
        self.worker.connect()

    def _disconnect(self):
        self.worker.disconnect()

    def _reconnect(self):
        self._update_target()
        self.worker.reconnect()

    def _scan_devices(self):
        """Gọi worker để scan thiết bị BLE."""
        self.worker.scan_devices(timeout=8.0)

    def _on_device_select(self, event):
        """Xử lý khi người dùng chọn một thiết bị từ danh sách."""
        selection = self.device_listbox.curselection()
        if not selection:
            return
        idx = selection[0]
        selected_text = self.device_listbox.get(idx)
        
        # Lấy tên thiết bị từ text (phần trước " @ ")
        if " @ " in selected_text:
            name, addr = selected_text.split(" @ ", 1)
            self.var_name.set(name)
            self.var_addr.set(addr)
            self._log(f"Selected device: {name} @ {addr}")

    def _send(self):
        s = self.var_cmd.get()
        # hỗ trợ người dùng gõ \n trong GUI
        s = s.encode("utf-8").decode("unicode_escape")  # biến "\\n" -> "\n"
        data = s.encode("utf-8")
        self.worker.send(data, with_response=self.var_resp.get())

    def _toggle_auto_send(self):
        """Toggle auto-send on/off"""
        if self.auto_send_active:
            # Stop auto-send
            self.auto_send_active = False
            if self.auto_send_job:
                self.after_cancel(self.auto_send_job)
                self.auto_send_job = None
            self.btn_auto_send.configure(text="▶ Auto Send")
            self._log("Auto-send stopped.")
        else:
            # Start auto-send
            try:
                interval = int(self.var_interval.get())
                if interval < 10:
                    interval = 10  # Minimum 10ms
                    self.var_interval.set("10")
            except ValueError:
                self._log("❌ Invalid interval. Using 1000ms.")
                interval = 1000
                self.var_interval.set("1000")
            
            self.auto_send_active = True
            self.btn_auto_send.configure(text="⏹ Stop Auto")
            self._log(f"Auto-send started (interval: {interval}ms)")
            self._auto_send_tick()

    def _auto_send_tick(self):
        """Send command periodically"""
        if not self.auto_send_active:
            return
        
        # Send command
        self._send()
        
        # Schedule next tick
        try:
            interval = int(self.var_interval.get())
            if interval < 10:
                interval = 10
        except ValueError:
            interval = 1000
        
        self.auto_send_job = self.after(interval, self._auto_send_tick)

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.txt.insert("end", f"[{ts}] {msg}\n")
        self.txt.see("end")

    def _set_connected(self, is_connected: bool):
        if is_connected:
            self.var_conn.set("CONNECTED")
            self.lbl_conn.configure(foreground="green")
            self.btn_connect.configure(state="disabled")
            self.btn_disc.configure(state="normal")
            self.btn_reconn.configure(state="normal")
        else:
            self.var_conn.set("DISCONNECTED")
            self.lbl_conn.configure(foreground="red")
            self.btn_connect.configure(state="normal")
            self.btn_disc.configure(state="disabled")
            self.btn_reconn.configure(state="normal")

    def _update_device_list(self, devices: list):
        """Cập nhật danh sách thiết bị vào Listbox."""
        self.device_listbox.delete(0, "end")
        self.devices_dict.clear()
        
        for name, addr in devices:
            display_text = f"{name} @ {addr}"
            self.device_listbox.insert("end", display_text)
            self.devices_dict[name] = addr

    def _poll_ui_queue(self):
        # Xử lý TẤT CẢ gói có trong queue
        latest_imu = None
        latest_mag = None
        imu_received = 0
        
        try:
            while True:
                typ, payload = self.ui_q.get_nowait()
                
                if typ == "log":
                    self._log(payload)
                elif typ == "status":
                    self.var_status.set(payload)
                elif typ == "conn":
                    self._set_connected(bool(payload))
                elif typ == "devices":
                    self._update_device_list(payload)
                elif typ == "imu":
                    # Chỉ giữ gói mới nhất
                    latest_imu = payload
                    imu_received += 1
                elif typ == "mag":
                    # Chỉ giữ gói mới nhất
                    latest_mag = payload
        except queue.Empty:
            pass
        
        # Chỉ vẽ 1 lần với data mới nhất
        if latest_imu is not None:
            yaw, pitch, roll = latest_imu
            self.var_yaw.set(f"{yaw:.1f}°")
            self.var_pitch.set(f"{pitch:.1f}°")
            self.var_roll.set(f"{roll:.1f}°")
            self.visualizer.update(yaw, pitch, roll)
            self.data_rate_count += imu_received
        
        if latest_mag is not None:
            mag_x, mag_y, mag_z = latest_mag
            self.var_mag_x.set(f"{mag_x:.1f} μT")
            self.var_mag_y.set(f"{mag_y:.1f} μT")
            self.var_mag_z.set(f"{mag_z:.1f} μT")
            self.mag_visualizer.update(mag_x, mag_y, mag_z)
        
        # Cập nhật data rate mỗi giây
        current_time = time.time()
        elapsed = current_time - self.data_rate_time
        if elapsed >= 1.0:
            self.current_data_rate = self.data_rate_count / elapsed
            self.var_data_rate.set(f"{self.current_data_rate:.1f} Hz")
            self.data_rate_count = 0
            self.data_rate_time = current_time
        
        # ULTRA FAST poll: mỗi 1ms để latency thấp nhất
        self.after(1, self._poll_ui_queue)

    def _on_close(self):
        # Stop auto-send
        if self.auto_send_active:
            self.auto_send_active = False
            if self.auto_send_job:
                self.after_cancel(self.auto_send_job)
        try:
            self.worker.shutdown()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
