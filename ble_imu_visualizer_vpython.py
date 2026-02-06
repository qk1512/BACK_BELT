import asyncio
import threading
import queue
import time
import json
from bleak import BleakClient, BleakScanner
from vpython import box, arrow, vector, rate, canvas, label, wtext

# ===== NUS UUIDs (Nordic UART Service) =====
NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


class BleWorker:
    """BLE worker để nhận dữ liệu yaw, pitch, roll từ device"""
    def __init__(self):
        self.cmd_q = queue.Queue()
        self.data_q = queue.Queue(maxsize=500)  # Queue lớn cho high-speed data
        self.thread = None
        self.loop = None

        self.client: BleakClient | None = None
        self.connected_addr: str | None = None
        self.stop_flag = threading.Event()
        self.want_connected = False
        self.auto_reconnect = True

        self.device_name = "BACKBELT"
        self.device_address = ""
        
        # Statistics
        self.data_count = 0
        self.last_stats_time = time.time()

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
    
    def send(self, data: bytes, with_response: bool = False):
        """Send data to device"""
        self.cmd_q.put(("send", (data, with_response)))

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        print("[BLE] Worker started")
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

            if cmd == "stop":
                print("[BLE] Stopping...")
                await self._safe_disconnect()
                self.loop.call_soon_threadsafe(self.loop.stop)
                return
            elif cmd == "connect":
                await self._connect_flow()
            elif cmd == "disconnect":
                await self._safe_disconnect()
            elif cmd == "send":
                data, with_resp = payload
                await self._send_flow(data, with_resp)

            if self.auto_reconnect and self.want_connected:
                if not (self.client and self.client.is_connected):
                    await asyncio.sleep(0.5)
                    await self._connect_flow()

            await asyncio.sleep(0.05)

    async def _find_by_name(self, name: str, timeout: float = 8.0):
        print(f"[BLE] Scanning for '{name}'...")
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            if d.name and name.upper() in d.name.upper():
                print(f"[BLE] Found: {d.name} @ {d.address}")
                return d.address
        return None

    def _on_nus_notify(self, sender: int, data: bytearray):
        raw = bytes(data)
        try:
            txt = raw.decode("utf-8", errors="strict").strip()
            self._parse_imu_data(txt)
        except Exception as e:
            pass

    def _parse_imu_data(self, txt: str):
        """Parse IMU data from various formats"""
        try:
            # Try JSON format first
            if txt.strip().startswith('{'):
                data = json.loads(txt)
                
                # Check for nested sensor format
                if 'sensor' in data and isinstance(data['sensor'], dict):
                    sensor = data['sensor']
                    if 'ypr' in sensor and isinstance(sensor['ypr'], list) and len(sensor['ypr']) >= 3:
                        roll = float(sensor['ypr'][0])
                        pitch = float(sensor['ypr'][1])
                        yaw = float(sensor['ypr'][2])
                        self._send_imu_data(yaw, pitch, roll)
                        return
                
                # Check for direct ypr format
                if 'ypr' in data and isinstance(data['ypr'], list) and len(data['ypr']) >= 3:
                    roll =float(data['ypr'][0])
                    pitch = float(data['ypr'][1])
                    yaw = float(data['ypr'][2])
                    self._send_imu_data(yaw, pitch, roll)
                    return
            
            # Try comma-separated format
            if ',' in txt:
                parts = txt.split(',')
                if len(parts) >= 3:
                    yaw = float(parts[0].strip())
                    pitch = float(parts[1].strip())
                    roll = float(parts[2].strip())
                    self._send_imu_data(yaw, pitch, roll)
                    return
                    
        except Exception:
            pass

    def _send_imu_data(self, yaw: float, pitch: float, roll: float):
        """Send IMU data to main thread"""
        self.data_count += 1
        try:
            self.data_q.put_nowait((yaw, pitch, roll))
        except queue.Full:
            # Queue full, skip this packet
            pass
        
        # Print statistics
        current_time = time.time()
        if current_time - self.last_stats_time >= 2.0:
            rate = self.data_count / (current_time - self.last_stats_time)
            print(f"[BLE] Data rate: {rate:.1f} Hz")
            self.data_count = 0
            self.last_stats_time = current_time

    def _on_disconnect_cb(self, client: BleakClient):
        print("[BLE] Disconnected from device")

    async def _connect_flow(self):
        if self.client and self.client.is_connected:
            return

        addr = self.device_address if self.device_address else None
        if not addr:
            if not self.device_name:
                print("[BLE] No target device specified")
                return
            addr = await self._find_by_name(self.device_name, timeout=8.0)

        if not addr:
            print("[BLE] Device not found")
            return

        print(f"[BLE] Connecting to {addr}...")

        try:
            self.client = BleakClient(addr, disconnected_callback=self._on_disconnect_cb)
            await self.client.connect(timeout=10.0)
            self.connected_addr = addr

            if not self.client.is_connected:
                raise RuntimeError("Connect failed")

            await self.client.start_notify(NUS_TX_UUID, self._on_nus_notify)
            print("[BLE] ✅ Connected and subscribed to notifications")

        except Exception as e:
            print(f"[BLE] Connect error: {e}")
            await self._safe_disconnect()

    async def _safe_disconnect(self):
        if not self.client:
            return
        try:
            if self.client.is_connected:
                try:
                    await self.client.stop_notify(NUS_TX_UUID)
                except Exception:
                    pass
                await self.client.disconnect()
        except Exception:
            pass
        finally:
            self.client = None
            self.connected_addr = None
    
    async def _send_flow(self, data: bytes, with_response: bool):
        """Send data to device"""
        if not (self.client and self.client.is_connected):
            print("[BLE] Not connected. Cannot send.")
            return
        try:
            await self.client.write_gatt_char(NUS_RX_UUID, data, response=with_response)
            print(f"[BLE] Sent: {data!r}")
        except Exception as e:
            print(f"[BLE] Send error: {e}")


def main():
    """Main application with VPython visualization"""
    
    # Create VPython scene
    scene = canvas(title="BLE IMU Visualizer - VPython (Ultra Fast)", 
                   width=1200, height=800, 
                   center=vector(0, 0, 0),
                   background=vector(0.1, 0.1, 0.15))
    
    # Create IMU box
    imu_box = box(pos=vector(0, 0, 0),
                  length=2, height=0.8, width=0.4,
                  color=vector(0, 0.7, 1))
    
    # Create axes arrows
    x_arrow = arrow(pos=vector(0, 0, 0), axis=vector(2, 0, 0),
                   color=vector(1, 0, 0), shaftwidth=0.1)
    y_arrow = arrow(pos=vector(0, 0, 0), axis=vector(0, 2, 0),
                   color=vector(0, 1, 0), shaftwidth=0.1)
    z_arrow = arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 2),
                   color=vector(0, 0, 1), shaftwidth=0.1)
    
    # Create labels
    status_label = label(pos=vector(0, 3, 0), text='Status: Initializing...', 
                        height=16, color=vector(1, 1, 1), box=False)
    data_label = label(pos=vector(0, 2.5, 0), text='Yaw: 0° Pitch: 0° Roll: 0°',
                      height=14, color=vector(0.8, 0.8, 1), box=False)
    fps_label = label(pos=vector(0, 2, 0), text='FPS: 0',
                     height=12, color=vector(0.5, 1, 0.5), box=False)
    
    # Instructions
    instruction_text = wtext(text="""<b>BLE IMU Visualizer - VPython Ultra Fast Mode</b><br>
Device: BACKBELT | Auto-connect: Enabled<br>
<span style="color: cyan">● Cyan Box</span> = IMU sensor<br>
<span style="color: red">→ Red</span> = X axis | 
<span style="color: green">→ Green</span> = Y axis | 
<span style="color: blue">→ Blue</span> = Z axis<br>
Rotate view: Right-click drag | Zoom: Scroll wheel<br><br>
<b style="color: yellow">Keyboard Controls (in terminal):</b><br>
<b>1</b> = 10 Hz | <b>2</b> = 20 Hz | <b>5</b> = 50 Hz | <b>0</b> = 100 Hz<br>
<b>s</b> = Start streaming | <b>p</b> = Stop streaming<br>
<b>c</b> = Calibrate | <b>r</b> = Reset | <b>q</b> = Quit
""")
    
    # Start BLE worker
    worker = BleWorker()
    worker.set_target("BACKBELT", "")
    worker.set_auto_reconnect(True)
    worker.start()
    worker.connect()
    
    status_label.text = 'Status: Connecting to BACKBELT...'
    
    # Rotation matrix function
    import math
    def rotation_matrix(yaw, pitch, roll):
        """Create rotation matrix from Euler angles (in degrees)"""
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        
        # ZYX order
        return [
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ]
    
    def apply_rotation(vec, R):
        """Apply rotation matrix to vector"""
        x = R[0][0]*vec.x + R[0][1]*vec.y + R[0][2]*vec.z
        y = R[1][0]*vec.x + R[1][1]*vec.y + R[1][2]*vec.z
        z = R[2][0]*vec.x + R[2][1]*vec.y + R[2][2]*vec.z
        return vector(x, y, z)
    
    # Main loop
    frame_count = 0
    last_fps_time = time.time()
    fps = 0
    
    # Keyboard input handling (non-blocking)
    import sys
    import select
    
    def check_keyboard():
        """Check for keyboard input (non-blocking)"""
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
        return None
    
    print("\n" + "="*60)
    print("BLE IMU Visualizer with VPython - ULTRA FAST MODE")
    print("="*60)
    print("Connecting to BACKBELT...")
    print("VPython scene opened in browser")
    print("")
    print("Keyboard Commands:")
    print("  1 = Set 10 Hz  | 2 = Set 20 Hz  | 5 = Set 50 Hz  | 0 = Set 100 Hz")
    print("  s = Start      | p = Stop       | c = Calibrate  | r = Reset")
    print("  q = Quit")
    print("")
    print("Current: Waiting for connection...\n")
    
    try:
        while True:
            rate(200)  # Max 200 FPS
            
            # Check keyboard input
            key = check_keyboard()
            if key:
                if key == 'q':
                    print("\n[Main] Quitting...")
                    break
                elif key == '1':
                    print("[CMD] Setting rate to 10 Hz")
                    worker.send(b"RATE:10\n")
                elif key == '2':
                    print("[CMD] Setting rate to 20 Hz")
                    worker.send(b"RATE:20\n")
                elif key == '5':
                    print("[CMD] Setting rate to 50 Hz")
                    worker.send(b"RATE:50\n")
                elif key == '0':
                    print("[CMD] Setting rate to 100 Hz")
                    worker.send(b"RATE:100\n")
                elif key == 's':
                    print("[CMD] Start streaming")
                    worker.send(b"START\n")
                elif key == 'p':
                    print("[CMD] Stop streaming")
                    worker.send(b"STOP\n")
                elif key == 'c':
                    print("[CMD] Calibrate sensor")
                    worker.send(b"CALIBRATE\n")
                elif key == 'r':
                    print("[CMD] Reset sensor")
                    worker.send(b"RESET\n")
            
            # Process all IMU data in queue (only keep latest)
            latest_data = None
            processed = 0
            max_process = 100
            
            while processed < max_process:
                try:
                    latest_data = worker.data_q.get_nowait()
                    processed += 1
                except queue.Empty:
                    break
            
            # Update visualization if we have new data
            if latest_data is not None:
                yaw, pitch, roll = latest_data
                
                # Create rotation matrix
                R = rotation_matrix(yaw, pitch, roll)
                
                # Update box orientation
                box_length = vector(2, 0, 0)
                box_height = vector(0, 0.8, 0)
                box_width = vector(0, 0, 0.4)
                
                rotated_length = apply_rotation(box_length, R)
                rotated_height = apply_rotation(box_height, R)
                rotated_width = apply_rotation(box_width, R)
                
                imu_box.axis = rotated_length
                imu_box.up = rotated_height
                
                # Update axes
                x_axis = apply_rotation(vector(2, 0, 0), R)
                y_axis = apply_rotation(vector(0, 2, 0), R)
                z_axis = apply_rotation(vector(0, 0, 2), R)
                
                x_arrow.axis = x_axis
                y_arrow.axis = y_axis
                z_arrow.axis = z_axis
                
                # Update labels
                data_label.text = f'Yaw: {yaw:.1f}° Pitch: {pitch:.1f}° Roll: {roll:.1f}°'
                status_label.text = 'Status: Connected ✓'
                
                frame_count += 1
            
            # Update FPS counter
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                fps_label.text = f'Render FPS: {fps:.1f}'
                frame_count = 0
                last_fps_time = current_time
                
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
        worker.shutdown()
        print("[Main] Bye!")


if __name__ == "__main__":
    main()
