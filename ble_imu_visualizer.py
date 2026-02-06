import asyncio
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
import json

from bleak import BleakClient, BleakScanner

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ===== NUS UUIDs (Nordic UART Service) =====
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class BleWorker:
    """
    BLE worker để nhận dữ liệu yaw, pitch, roll từ device
    """
    def __init__(self, ui_queue: queue.Queue):
        self.ui_q = ui_queue
        self.cmd_q = queue.Queue()
        self.thread = None
        self.loop = None
        self.last_data_time = 0
        self.data_count = 0

        self.client: BleakClient | None = None
        self.connected_addr: str | None = None

        self.stop_flag = threading.Event()
        self.want_connected = False
        self.auto_reconnect = True

        self.device_name = "BACKBELT"
        self.device_address = ""

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

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.stop_flag.clear()
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()

    def shutdown(self):
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
        self.want_connected = True
        self.cmd_q.put(("reconnect", None))

    def send(self, data: bytes, with_response: bool = False):
        self.cmd_q.put(("send", (data, with_response)))

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self._ui_status("BLE worker started.")
        self.loop.create_task(self._runner())
        try:
            self.loop.run_forever()
        finally:
            try:
                pending = asyncio.all_tasks(self.loop)
                for t in pending:
                    t.cancel()
                self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except Exception:
                pass
            self.loop.close()

    async def _runner(self):
        while not self.stop_flag.is_set():
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

            if self.auto_reconnect and self.want_connected:
                if not (self.client and self.client.is_connected):
                    await asyncio.sleep(0.5)
                    await self._connect_flow()

            await asyncio.sleep(0.05)

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
        try:
            txt = raw.decode("utf-8", errors="strict").strip()
            self._ui_log(f"[RX] {txt}")
            
            # Parse yaw, pitch, roll from text
            # Expected format: "yaw,pitch,roll" or "Y:yaw P:pitch R:roll"
            self._parse_imu_data(txt)
            
        except Exception as e:
            self._ui_log(f"Parse error: {e}")

    def _parse_imu_data(self, txt: str):
        """
        Parse IMU data from various formats:
        - JSON: {"sensor":{"time":78016,"status":true,"accel":[...],"gyro":[...],"ypr":[-12.27,-55.82,11.58]}}
        - JSON: {"time":31221,"accel":[0.02,0.24,-8.11],"gyro":[0.00,0.00,0.00],"ypr":[-31.54,-2.23,0.05]}
        - CSV: "yaw,pitch,roll"
        - Colon: "Y:yaw P:pitch R:roll"
        - Space: "yaw pitch roll"
        """
        try:
            # Try JSON format first
            if txt.strip().startswith('{'):
                data = json.loads(txt)
                
                # Check for nested sensor format: {"sensor":{"ypr":[...]}}
                if 'sensor' in data and isinstance(data['sensor'], dict):
                    sensor = data['sensor']
                    if 'ypr' in sensor and isinstance(sensor['ypr'], list) and len(sensor['ypr']) >= 3:
                        roll = float(sensor['ypr'][0])
                        pitch = float(sensor['ypr'][1])
                        yaw = float(sensor['ypr'][2])
                        self._ui_imu_data(yaw, pitch, roll)
                        return
                
                # Check for direct ypr format: {"ypr":[...]}
                if 'ypr' in data and isinstance(data['ypr'], list) and len(data['ypr']) >= 3:
                    roll = float(data['ypr'][0])
                    pitch = float(data['ypr'][1])
                    yaw = float(data['ypr'][2])
                    self._ui_imu_data(yaw, pitch, roll)
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
                
        except Exception as e:
            # Không parse được thì bỏ qua
            pass

    def _on_disconnect_cb(self, client: BleakClient):
        self._ui_log("⚠️ Disconnected from device.")
        self._ui_conn(False)
        self._ui_status("Disconnected.")

    async def _connect_flow(self):
        if self.client and self.client.is_connected:
            self._ui_status("Already connected.")
            self._ui_conn(True)
            return

        addr = self.device_address if self.device_address else None
        if not addr:
            if not self.device_name:
                self._ui_log("❌ Please input device name or address.")
                self._ui_status("No target.")
                self._ui_conn(False)
                return
            addr = await self._find_by_name(self.device_name, timeout=8.0)

        if not addr:
            self._ui_log("❌ Device not found.")
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

    async def _send_flow(self, data: bytes, with_response: bool):
        if not (self.client and self.client.is_connected):
            self._ui_log("❌ Not connected. Cannot send.")
            return
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=with_response)
            self._ui_log(f"[TX] Sent {len(data)} bytes: {data!r}")
        except Exception as e:
            self._ui_log(f"❌ Send error: {e}")


class IMUVisualizer:
    """
    Visualize IMU orientation in 3D using matplotlib
    """
    def __init__(self, parent_frame):
        # DPI cực thấp để render siêu nhanh
        self.fig = Figure(figsize=(6, 6), dpi=40)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Current orientation
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        
        # Giảm frame skipping để vẽ nhiều hơn = FPS cao hơn
        self.skip_counter = 0
        self.skip_every = 1  # Vẽ 1 trong 2 frames (50% render)
        
        # Cache cho legend (vẽ 1 lần)
        self.legend_drawn = False
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        
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
        Draw 3D box and axes arrows
        """
        self.ax.clear()
        self._setup_plot()
        
        # Box vertices - compact size
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
        
        # Chỉ vẽ 2 mặt chính (top, front) để tối ưu tối đa
        faces = [
            [rotated_vertices[4], rotated_vertices[5], rotated_vertices[6], rotated_vertices[7]],  # top
            [rotated_vertices[3], rotated_vertices[2], rotated_vertices[6], rotated_vertices[7]]   # front
        ]
        
        # Box polygon - NO EDGES để nhanh hơn
        poly3d = Poly3DCollection(faces, facecolor='cyan', alpha=0.4, edgecolor='none')
        self.ax.add_collection3d(poly3d)
        
        # Draw axes arrows
        axis_length = 1.2
        axes = np.array([
            [axis_length, 0, 0],
            [0, axis_length, 0],
            [0, 0, axis_length]
        ])
        rotated_axes = axes @ R.T
        
        # Arrows mỏng hơn
        self.ax.plot([0, rotated_axes[0, 0]], [0, rotated_axes[0, 1]], [0, rotated_axes[0, 2]], 
                     'r-', linewidth=1.2)
        self.ax.plot([0, rotated_axes[1, 0]], [0, rotated_axes[1, 1]], [0, rotated_axes[1, 2]], 
                     'g-', linewidth=1.2)
        self.ax.plot([0, rotated_axes[2, 0]], [0, rotated_axes[2, 1]], [0, rotated_axes[2, 2]], 
                     'b-', linewidth=1.2)
        
        # Legend vẽ 1 lần
        if not self.legend_drawn:
            self.ax.plot([], [], 'r-', linewidth=1.2, label='X')
            self.ax.plot([], [], 'g-', linewidth=1.2, label='Y')
            self.ax.plot([], [], 'b-', linewidth=1.2, label='Z')
            self.ax.legend(loc='upper right')
            self.legend_drawn = True
        
        # Draw
        self.canvas.draw_idle()
        self.canvas.flush_events()

    def update(self, yaw, pitch, roll, force=False):
        """
        Update orientation with reduced frame skipping for higher FPS
        """
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        
        # Frame skipping: vẽ 1 trong 2 frames
        self.skip_counter += 1
        if self.skip_counter <= self.skip_every:
            return  # Bỏ qua frame này
        
        self.skip_counter = 0
        
        # Vẽ frame thực sự
        self._draw_object()
        
        # Track FPS
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_fps_time >= 1.0:  # Mỗi 1s thay vì 2s
            fps = self.frame_count / (current_time - self.last_fps_time)
            print(f"[Render FPS: {fps:.1f}]")
            self.frame_count = 0
            self.last_fps_time = current_time


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("BLE IMU Visualizer - ULTRA HIGH FPS (DPI 40, 50% Render)")
        self.geometry("1100x700")

        # Real-time mode: queue lớn hơn để không mất gói tin
        self.ui_q = queue.Queue(maxsize=200)
        self.worker = BleWorker(self.ui_q)
        self.worker.start()
        
        # Tracking data rate
        self.data_rate_count = 0
        self.data_rate_time = time.time()
        self.current_data_rate = 0.0

        self._build_ui()
        self.after(1, self._poll_ui_queue)  # EXTREME FAST: poll mỗi 1ms

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        # Main container
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # Left panel for controls and log
        left_panel = ttk.Frame(main, width=400)
        left_panel.pack(side="left", fill="both", expand=False, padx=(0, 10))

        # Right panel for 3D visualization
        right_panel = ttk.Frame(main)
        right_panel.pack(side="right", fill="both", expand=True)

        # --- Connection controls ---
        conn_frame = ttk.LabelFrame(left_panel, text="Connection", padding=10)
        conn_frame.pack(fill="x")

        ttk.Label(conn_frame, text="Device name:").grid(row=0, column=0, sticky="w")
        self.var_name = tk.StringVar(value="BACKBELT")
        ttk.Entry(conn_frame, textvariable=self.var_name, width=20).grid(row=0, column=1, sticky="w", pady=5)

        ttk.Label(conn_frame, text="Address:").grid(row=1, column=0, sticky="w")
        self.var_addr = tk.StringVar(value="")
        ttk.Entry(conn_frame, textvariable=self.var_addr, width=20).grid(row=1, column=1, sticky="w", pady=5)

        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn_frame, text="Auto reconnect", variable=self.var_auto,
                       command=self._apply_settings).grid(row=2, column=0, columnspan=2, sticky="w", pady=5)

        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=3, column=0, columnspan=2, pady=10)

        self.btn_connect = ttk.Button(btn_frame, text="Connect", command=self._connect)
        self.btn_disc = ttk.Button(btn_frame, text="Disconnect", command=self._disconnect)
        self.btn_reconn = ttk.Button(btn_frame, text="Reconnect", command=self._reconnect)

        self.btn_connect.pack(side="left", padx=2)
        self.btn_disc.pack(side="left", padx=2)
        self.btn_reconn.pack(side="left", padx=2)

        # Status
        status_frame = ttk.Frame(conn_frame)
        status_frame.grid(row=4, column=0, columnspan=2, pady=5)

        ttk.Label(status_frame, text="Status:").pack(side="left")
        self.var_status = tk.StringVar(value="Idle")
        ttk.Label(status_frame, textvariable=self.var_status, foreground="blue").pack(side="left", padx=5)

        self.var_conn = tk.StringVar(value="DISCONNECTED")
        self.lbl_conn = ttk.Label(status_frame, textvariable=self.var_conn, foreground="red")
        self.lbl_conn.pack(side="left", padx=10)

        # --- IMU data display ---
        imu_frame = ttk.LabelFrame(left_panel, text="IMU Data", padding=10)
        imu_frame.pack(fill="x", pady=(10, 0))

        ttk.Label(imu_frame, text="Yaw:").grid(row=0, column=0, sticky="w", pady=3)
        self.var_yaw = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_yaw, width=12, font=("TkDefaultFont", 12, "bold")).grid(
            row=0, column=1, sticky="w", pady=3)

        ttk.Label(imu_frame, text="Pitch:").grid(row=1, column=0, sticky="w", pady=3)
        self.var_pitch = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_pitch, width=12, font=("TkDefaultFont", 12, "bold")).grid(
            row=1, column=1, sticky="w", pady=3)

        ttk.Label(imu_frame, text="Roll:").grid(row=2, column=0, sticky="w", pady=3)
        self.var_roll = tk.StringVar(value="0.0°")
        ttk.Label(imu_frame, textvariable=self.var_roll, width=12, font=("TkDefaultFont", 12, "bold")).grid(
            row=2, column=1, sticky="w", pady=3)
        
        ttk.Label(imu_frame, text="Data Rate:").grid(row=3, column=0, sticky="w", pady=3)
        self.var_data_rate = tk.StringVar(value="0 Hz")
        ttk.Label(imu_frame, textvariable=self.var_data_rate, width=12, foreground="blue").grid(
            row=3, column=1, sticky="w", pady=3)

        # --- Send command ---
        cmd_frame = ttk.LabelFrame(left_panel, text="Send Command", padding=10)
        cmd_frame.pack(fill="x", pady=(10, 0))

        ttk.Label(cmd_frame, text="Command:").grid(row=0, column=0, sticky="w", pady=3)
        self.var_cmd = tk.StringVar(value="hello\\n")
        ttk.Entry(cmd_frame, textvariable=self.var_cmd, width=25).grid(
            row=0, column=1, sticky="we", pady=3, padx=5)

        self.var_resp = tk.BooleanVar(value=False)
        ttk.Checkbutton(cmd_frame, text="With response", variable=self.var_resp).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=3)

        ttk.Button(cmd_frame, text="Send", command=self._send).grid(
            row=2, column=0, columnspan=2, sticky="we", pady=5)

        cmd_frame.grid_columnconfigure(1, weight=1)

        # --- Log area ---
        log_frame = ttk.LabelFrame(left_panel, text="Log", padding=10)
        log_frame.pack(fill="both", expand=True, pady=(10, 0))

        self.txt = tk.Text(log_frame, wrap="word", height=15, width=45)
        self.txt.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(log_frame, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)

        # --- 3D Visualization ---
        viz_frame = ttk.LabelFrame(right_panel, text="3D Orientation", padding=10)
        viz_frame.pack(fill="both", expand=True)

        self.visualizer = IMUVisualizer(viz_frame)

        self._log("Ready. Connect to device to visualize IMU data.")
        self._log("Expected data format: JSON {'ypr':[yaw,pitch,roll]} or 'yaw,pitch,roll'")
        self._log("⚡ULTRA HIGH FPS: DPI 40, 50% render, 2 faces, no edges, 1ms poll")
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

    def _send(self):
        s = self.var_cmd.get()
        # Hỗ trợ người dùng gõ \n trong GUI
        s = s.encode("utf-8").decode("unicode_escape")  # biến "\\n" -> "\n"
        data = s.encode("utf-8")
        self.worker.send(data, with_response=self.var_resp.get())

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

    def _poll_ui_queue(self):
        # Xử lý TẤT CẢ gói có trong queue, không giới hạn
        latest_imu = None
        imu_received = 0
        
        try:
            # ĐỌNG VÒNG LẶP cho đến khi queue rỗng
            while True:
                typ, payload = self.ui_q.get_nowait()
                
                if typ == "log":
                    self._log(payload)
                elif typ == "status":
                    self.var_status.set(payload)
                elif typ == "conn":
                    self._set_connected(bool(payload))
                elif typ == "imu":
                    # Chỉ giữ gói mới nhất
                    latest_imu = payload
                    imu_received += 1
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
        
        # Cập nhật data rate mỗi giây
        current_time = time.time()
        elapsed = current_time - self.data_rate_time
        if elapsed >= 1.0:
            self.current_data_rate = self.data_rate_count / elapsed
            self.var_data_rate.set(f"{self.current_data_rate:.1f} Hz")
            self.data_rate_count = 0
            self.data_rate_time = current_time
        
        # EXTREME FAST: poll mỗi 2ms để giảm lag
        self.after(2, self._poll_ui_queue)

    def _on_close(self):
        try:
            self.worker.shutdown()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
