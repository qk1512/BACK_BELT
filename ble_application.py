import asyncio
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk

import csv
import os
from datetime import datetime

from bleak import BleakClient, BleakScanner

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

        self.device_name = "left_insole"
        self.device_address = ""      # nếu set thì ưu tiên connect theo address

        # ===== CSV logging =====
        self.csv_enabled = True
        self.csv_path = "logs/nus_log.csv"
        self._csv_fp = None
        self._csv_writer = None
        self._csv_lock = threading.Lock()

    # ---------- UI helpers ----------
    def _ui_log(self, msg: str):
        self.ui_q.put(("log", msg))

    def _ui_status(self, msg: str):
        self.ui_q.put(("status", msg))

    def _ui_conn(self, is_connected: bool):
        self.ui_q.put(("conn", is_connected))

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

        self._csv_fp = open(self.csv_path, "a", newline="", encoding="utf-8")
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

            # auto reconnect nếu đang “muốn kết nối” nhưng bị rớt
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

        line = f"[NUS TX] {hexs} | len={len(raw)}"
        try:
            txt = raw.decode("utf-8", errors="strict").strip()
            line += f" | text='{txt}'"
        except Exception:
            pass

        self._ui_log(line)

        # ===== Save to CSV =====
        if self.csv_enabled:
            with self._csv_lock:
                try:
                    self._csv_open_if_needed()
                    ts_iso = datetime.now().isoformat(timespec="milliseconds")
                    self._csv_writer.writerow([ts_iso, txt])
                    self._csv_fp.flush()
                except Exception as e:
                    self._ui_log(f"❌ CSV write error: {e}")

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
                # NOTE: có thể fail ở một số version/OS, bỏ qua vẫn OK
                await self.client.get_services()
            except Exception:
                pass

            # kiểm tra NUS service (optional)
            has_nus = False
            try:
                svcs = self.client.services
                if svcs:
                    has_nus = any(s.uuid.lower() == NUS_SERVICE_UUID.lower() for s in svcs)
            except Exception:
                pass

            self._ui_log(f"NUS service found: {has_nus}")

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
            self._ui_log(f"[NUS RX] sent {data!r} (response={with_response})")
        except Exception as e:
            self._ui_log(f"❌ Send error: {e}")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("NUS BLE GUI (Bleak + Tkinter) + CSV")
        self.geometry("900x560")

        self.ui_q = queue.Queue()
        self.worker = BleWorker(self.ui_q)

        # Set CSV file name by timestamp
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.worker.set_csv(True, f"logs/nus_{ts}.csv")

        self.worker.start()

        self._build_ui()
        self.after(80, self._poll_ui_queue)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # --- Top controls ---
        top = ttk.LabelFrame(main, text="Connection", padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Device name:").grid(row=0, column=0, sticky="w")
        self.var_name = tk.StringVar(value="BACKBELT")
        ttk.Entry(top, textvariable=self.var_name, width=26).grid(row=0, column=1, sticky="w", padx=6)

        ttk.Label(top, text="Device address (optional):").grid(row=0, column=2, sticky="w", padx=(20, 0))
        self.var_addr = tk.StringVar(value="")
        ttk.Entry(top, textvariable=self.var_addr, width=34).grid(row=0, column=3, sticky="w", padx=6)

        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Auto reconnect", variable=self.var_auto, command=self._apply_settings)\
            .grid(row=0, column=4, sticky="w", padx=(20, 0))

        self.btn_scan    = ttk.Button(top, text="Scan Devices", command=self._scan_devices)
        self.btn_connect = ttk.Button(top, text="Connect", command=self._connect)
        self.btn_disc    = ttk.Button(top, text="Disconnect", command=self._disconnect)
        self.btn_reconn   = ttk.Button(top, text="Reconnect", command=self._reconnect)

        self.btn_scan.grid(row=1, column=0, pady=(10, 0), sticky="w")
        self.btn_connect.grid(row=1, column=1, pady=(10, 0), sticky="w")
        self.btn_disc.grid(row=1, column=2, pady=(10, 0), sticky="w")
        self.btn_reconn.grid(row=1, column=3, pady=(10, 0), sticky="w")

        ttk.Label(top, text="Status:").grid(row=1, column=4, pady=(10, 0), sticky="e")
        self.var_status = tk.StringVar(value="Idle")
        ttk.Label(top, textvariable=self.var_status).grid(row=1, column=5, pady=(10, 0), sticky="w")

        self.var_conn = tk.StringVar(value="DISCONNECTED")
        self.lbl_conn = ttk.Label(top, textvariable=self.var_conn)
        self.lbl_conn.grid(row=1, column=6, pady=(10, 0), sticky="w", padx=(20, 0))

        for c in range(7):
            top.grid_columnconfigure(c, weight=0)
        top.grid_columnconfigure(4, weight=1)

        # --- Send command ---
        cmdf = ttk.LabelFrame(main, text="Send command (NUS RX)", padding=10)
        cmdf.pack(fill="x", pady=(10, 0))

        ttk.Label(cmdf, text="Command:").grid(row=0, column=0, sticky="w")
        self.var_cmd = tk.StringVar(value="hello from pc\\n")
        ttk.Entry(cmdf, textvariable=self.var_cmd, width=70).grid(row=0, column=1, sticky="we", padx=6)

        self.var_resp = tk.BooleanVar(value=False)
        ttk.Checkbutton(cmdf, text="Write with response", variable=self.var_resp)\
            .grid(row=0, column=2, sticky="w", padx=(10, 0))

        ttk.Button(cmdf, text="Send", command=self._send).grid(row=0, column=3, sticky="w", padx=(10, 0))

        cmdf.grid_columnconfigure(1, weight=1)

        # --- Discovered Devices ---
        devf = ttk.LabelFrame(main, text="Discovered Devices", padding=10)
        devf.pack(fill="both", expand=True, pady=(10, 0))

        # Frame chứa listbox và scrollbar
        list_frame = ttk.Frame(devf)
        list_frame.pack(fill="both", expand=True)

        self.device_listbox = tk.Listbox(list_frame, height=6)
        self.device_listbox.pack(side="left", fill="both", expand=True)
        self.device_listbox.bind("<<ListboxSelect>>", self._on_device_select)

        dev_sb = ttk.Scrollbar(list_frame, command=self.device_listbox.yview)
        dev_sb.pack(side="right", fill="y")
        self.device_listbox.configure(yscrollcommand=dev_sb.set)

        # Dictionary để lưu thông tin thiết bị (name -> address)
        self.devices_dict = {}

        # --- Log area ---
        logf = ttk.LabelFrame(main, text="Log", padding=10)
        logf.pack(fill="both", expand=True, pady=(10, 0))

        self.txt = tk.Text(logf, wrap="word", height=18)
        self.txt.pack(side="left", fill="both", expand=True)

        sb = ttk.Scrollbar(logf, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)

        # hint
        self._log(f"Ready. CSV is saving to: {self.worker.csv_path}")
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

    def _log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.txt.insert("end", f"[{ts}] {msg}\n")
        self.txt.see("end")

    def _set_connected(self, is_connected: bool):
        if is_connected:
            self.var_conn.set("CONNECTED")
            self.btn_connect.configure(state="disabled")
            self.btn_disc.configure(state="normal")
            self.btn_reconn.configure(state="normal")
        else:
            self.var_conn.set("DISCONNECTED")
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
        except queue.Empty:
            pass
        self.after(80, self._poll_ui_queue)

    def _on_close(self):
        try:
            self.worker.shutdown()
        except Exception:
            pass
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
