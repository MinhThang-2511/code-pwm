import time
import threading
import datetime
import customtkinter as ctk
import serial
import serial.tools.list_ports

try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


class MultiMotorCockpitApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Aircraft Quad-Motor Cockpit Control - USB UART PWM")
        self.geometry("1100x680")
        self.resizable(False, False)
        self.configure(fg_color="#090A0F")

        # --- BIẾN TRẠNG THÁI HỆ THỐNG ---
        self.serial_port = None
        self.is_connected = False
        self.connected_port_name = ""
        self.running = True
        self.last_sent_time = 0

        # Trạng thái từng động cơ: Motor ID (0 = ALL, 1=M1, 2=M2, 3=M3, 4=M4)
        self.selected_motor_id = 0  # 0: Cả 4 động cơ, 1..4: Động cơ riêng biệt
        self.motor_throttles = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        self.motor_directions = {1: False, 2: False, 3: False, 4: False} # False = FWD, True = REV

        self._build_ui()

        # Threads
        self.transmit_thread = threading.Thread(target=self._transmit_loop, daemon=True)
        self.transmit_thread.start()

        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

        self.watchdog_thread = threading.Thread(target=self._hardware_watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self._refresh_ports()
        self._add_log("Hệ thống Cockpit 4 Động cơ đã sẵn sàng.")

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        # 1. Header Bar
        self.header_frame = ctk.CTkFrame(self, fg_color="#12151E", corner_radius=15, border_width=1, border_color="#1E2330")
        self.header_frame.pack(fill="x", padx=15, pady=12)

        self.status_indicator = ctk.CTkLabel(
            self.header_frame, text="● DISCONNECTED", text_color="#FF2A6D", font=ctk.CTkFont(size=13, weight="bold")
        )
        self.status_indicator.pack(side="left", padx=15, pady=10)

        self.port_option = ctk.CTkOptionMenu(
            self.header_frame,
            values=["No Ports Detected"],
            fg_color="#090A0F",
            button_color="#1C2230",
            button_hover_color="#00E5FF",
            dropdown_fg_color="#12151E",
            width=380,
        )
        self.port_option.pack(side="left", fill="x", expand=True, padx=10, pady=10)

        self.btn_refresh = ctk.CTkButton(
            self.header_frame, text="Refresh", width=80, fg_color="#1C2230", hover_color="#2E3545", command=self._refresh_ports
        )
        self.btn_refresh.pack(side="left", padx=5, pady=10)

        self.btn_connect = ctk.CTkButton(
            self.header_frame,
            text="CONNECT",
            width=110,
            fg_color="#00E5FF",
            text_color="#000000",
            hover_color="#00B8D4",
            font=ctk.CTkFont(weight="bold"),
            command=self._toggle_connection,
        )
        self.btn_connect.pack(side="left", padx=(5, 15), pady=10)

        # Body Layout
        self.body_frame = ctk.CTkFrame(self, fg_color="transparent")
        self.body_frame.pack(fill="both", expand=True, padx=15, pady=(0, 15))

        # 2. Panel Chọn Động cơ & Gauge
        self.left_panel = ctk.CTkFrame(self.body_frame, fg_color="#12151E", corner_radius=20, border_width=1, border_color="#1E2330")
        self.left_panel.pack(side="left", fill="both", expand=True, padx=(0, 8))

        # Bảng chọn động cơ (Motor Selection Grid)
        self.selector_label = ctk.CTkLabel(
            self.left_panel, text="BẢNG CHỌN ĐỘNG CƠ", text_color="#8A8F9E", font=ctk.CTkFont(size=11, weight="bold")
        )
        self.selector_label.pack(pady=(12, 5))

        self.motor_btn_frame = ctk.CTkFrame(self.left_panel, fg_color="transparent")
        self.motor_btn_frame.pack(fill="x", padx=15, pady=5)

        self.btn_motors = {}
        motors_info = [("ALL MOTORS", 0), ("M1", 1), ("M2", 2), ("M3", 3), ("M4", 4)]
        
        for text, m_id in motors_info:
            btn = ctk.CTkButton(
                self.motor_btn_frame,
                text=text,
                width=55 if m_id != 0 else 90,
                height=32,
                fg_color="#00E5FF" if m_id == 0 else "#1C2230",
                text_color="#000000" if m_id == 0 else "#FFFFFF",
                hover_color="#00B8D4",
                font=ctk.CTkFont(size=11, weight="bold"),
                command=lambda id=m_id: self._select_motor(id)
            )
            btn.pack(side="left", padx=3)
            self.btn_motors[m_id] = btn

        # Canvas Arc Gauge
        self.gauge_canvas = ctk.CTkCanvas(self.left_panel, width=220, height=170, bg="#12151E", highlightthickness=0)
        self.gauge_canvas.pack(pady=(5, 0))

        # Action Buttons
        self.btn_dir = ctk.CTkButton(
            self.left_panel,
            text="FORWARD DIRECTION",
            height=38,
            fg_color="#1C2230",
            hover_color="#2E3545",
            border_width=1,
            border_color="#00E5FF",
            font=ctk.CTkFont(weight="bold"),
            command=self._toggle_direction,
        )
        self.btn_dir.pack(fill="x", padx=15, pady=(5, 5))

        self.btn_estop = ctk.CTkButton(
            self.left_panel,
            text="EMERGENCY STOP (ALL)",
            height=38,
            fg_color="#D32F2F",
            hover_color="#9A0007",
            font=ctk.CTkFont(size=12, weight="bold"),
            command=self._emergency_stop,
        )
        self.btn_estop.pack(fill="x", padx=15, pady=(0, 12))

        # 3. Middle Panel: Throttle Lever
        self.mid_panel = ctk.CTkFrame(self.body_frame, fg_color="#12151E", corner_radius=20, border_width=1, border_color="#1E2330", width=200)
        self.mid_panel.pack(side="left", fill="both", padx=8)

        self.throttle_label = ctk.CTkLabel(
            self.mid_panel, text="THROTTLE LEVER", text_color="#8A8F9E", font=ctk.CTkFont(size=12, weight="bold")
        )
        self.throttle_label.pack(pady=(15, 5))

        self.throttle_canvas = ctk.CTkCanvas(self.mid_panel, width=160, height=280, bg="#12151E", highlightthickness=0)
        self.throttle_canvas.pack(pady=5)
        self.throttle_canvas.bind("<B1-Motion>", self._on_throttle_drag)
        self.throttle_canvas.bind("<Button-1>", self._on_throttle_drag)

        self.throttle_val_label = ctk.CTkLabel(
            self.mid_panel, text="0%", text_color="#00E5FF", font=ctk.CTkFont(size=22, weight="bold")
        )
        self.throttle_val_label.pack(pady=(5, 10))

        # 4. Right Panel: Log & Telemetry Table
        self.right_panel = ctk.CTkFrame(self.body_frame, fg_color="#12151E", corner_radius=20, border_width=1, border_color="#1E2330")
        self.right_panel.pack(side="right", fill="both", expand=True, padx=(8, 0))

        # Bảng hiển thị thông số 4 động cơ
        self.telemetry_frame = ctk.CTkFrame(self.right_panel, fg_color="#090A0F", corner_radius=10, border_width=1, border_color="#1E2330")
        self.telemetry_frame.pack(fill="x", padx=12, pady=(12, 5))

        self.motor_status_labels = {}
        for m_id in range(1, 5):
            f = ctk.CTkFrame(self.telemetry_frame, fg_color="transparent")
            f.pack(fill="x", padx=8, pady=3)
            
            lbl_title = ctk.CTkLabel(f, text=f"MOTOR {m_id}:", text_color="#8A8F9E", font=ctk.CTkFont(size=11, weight="bold"), width=70, anchor="w")
            lbl_title.pack(side="left")
            
            lbl_val = ctk.CTkLabel(f, text="0% | PWM: 0 | FWD", text_color="#00E5FF", font=ctk.CTkFont(family="monospace", size=11))
            lbl_val.pack(side="right")
            self.motor_status_labels[m_id] = lbl_val

        self.log_header = ctk.CTkFrame(self.right_panel, fg_color="transparent")
        self.log_header.pack(fill="x", padx=12, pady=(10, 2))

        self.log_label = ctk.CTkLabel(
            self.log_header, text="UART LOG", text_color="#00E5FF", font=ctk.CTkFont(size=11, weight="bold")
        )
        self.log_label.pack(side="left")

        self.btn_clear_log = ctk.CTkButton(
            self.log_header, text="Clear", width=45, height=20, fg_color="#1C2230", hover_color="#2E3545", font=ctk.CTkFont(size=10), command=self._clear_log
        )
        self.btn_clear_log.pack(side="right")

        self.log_textbox = ctk.CTkTextbox(
            self.right_panel,
            fg_color="#090A0F",
            text_color="#00FF66",
            font=ctk.CTkFont(family="monospace", size=10),
            border_width=1,
            border_color="#1E2330",
            corner_radius=10,
        )
        self.log_textbox.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        self._update_ui_state()

    # --- CHỌN VÀ CẬP NHẬT ĐỘNG CƠ ---
    def _select_motor(self, motor_id):
        self.selected_motor_id = motor_id
        for m_id, btn in self.btn_motors.items():
            if m_id == motor_id:
                btn.configure(fg_color="#00E5FF", text_color="#000000")
            else:
                btn.configure(fg_color="#1C2230", text_color="#FFFFFF")
        
        target_str = "CẢ 4 ĐỘNG CƠ" if motor_id == 0 else f"ĐỘNG CƠ M{motor_id}"
        self._add_log(f"[SELECT] Chuyển sang điều khiển: {target_str}")
        self._update_ui_state()

    def _get_current_throttle(self):
        if self.selected_motor_id == 0:
            return self.motor_throttles[1]
        return self.motor_throttles[self.selected_motor_id]

    def _get_current_direction(self):
        if self.selected_motor_id == 0:
            return self.motor_directions[1]
        return self.motor_directions[self.selected_motor_id]

    def _update_ui_state(self):
        # Cập nhật nút Hướng
        is_rev = self._get_current_direction()
        if is_rev:
            self.btn_dir.configure(text="REVERSE DIRECTION", fg_color="#FF2A6D", border_color="#FF2A6D")
        else:
            self.btn_dir.configure(text="FORWARD DIRECTION", fg_color="#1C2230", border_color="#00E5FF")

        # Cập nhật Telemetry Table
        for m_id in range(1, 5):
            th = self.motor_throttles[m_id]
            pwm = int((th / 100.0) * 255)
            d_str = "REV" if self.motor_directions[m_id] else "FWD"
            self.motor_status_labels[m_id].configure(text=f"{int(th):3d}% | PWM: {pwm:3d} | {d_str}")

        self._draw_gauge()
        self._draw_throttle()

    # --- KẾT NỐI UART & WATCHDOG ---
    def _refresh_ports(self):
        ports_info = serial.tools.list_ports.comports()
        self.ports_mapping = {}
        display_list = []

        for port in ports_info:
            desc = f"{port.device} - {port.description}"
            self.ports_mapping[desc] = port.device
            display_list.append(desc)

        if not display_list:
            display_list = ["No Ports Detected"]

        self.port_option.configure(values=display_list)
        self.port_option.set(display_list[0])

    def _toggle_connection(self):
        if not self.is_connected:
            selected_display = self.port_option.get()
            if selected_display == "No Ports Detected":
                self._add_log("[ERROR] Không có cổng COM nào!")
                return

            port_name = self.ports_mapping.get(selected_display, selected_display.split(" ")[0])

            try:
                self.serial_port = serial.Serial(port_name, 115200, timeout=0.1)
                self.is_connected = True
                self.connected_port_name = port_name
                self.status_indicator.configure(text=f"● CONNECTED ({port_name})", text_color="#00E5FF")
                self.btn_connect.configure(text="DISCONNECT", fg_color="#FF2A6D", hover_color="#C70039", text_color="#FFFFFF")
                self._add_log(f"[CONNECTED] {port_name} (115200 Baud)")
            except Exception as e:
                self._add_log(f"[ERROR] Không thể mở cổng {port_name}: {e}")
        else:
            self._disconnect_port(reason="Người dùng chủ động ngắt kết nối")

    def _disconnect_port(self, reason="Mất kết nối"):
        if self.is_connected:
            self.is_connected = False
            if self.serial_port:
                try:
                    if self.serial_port.is_open:
                        self.serial_port.close()
                except Exception:
                    pass
                self.serial_port = None
            self.after(0, self._update_ui_on_disconnect, reason)

    def _update_ui_on_disconnect(self, reason):
        self.status_indicator.configure(text="● DISCONNECTED", text_color="#FF2A6D")
        self.btn_connect.configure(text="CONNECT", fg_color="#00E5FF", hover_color="#00B8D4", text_color="#000000")
        self._add_log(f"[DISCONNECTED] {reason}.")

    def _hardware_watchdog_loop(self):
        while self.running:
            if self.is_connected and self.connected_port_name:
                active_ports = [p.device for p in serial.tools.list_ports.comports()]
                if self.connected_port_name not in active_ports:
                    if HAS_WINSOUND:
                        try:
                            winsound.Beep(1000, 250)
                        except Exception:
                            pass
                    self._disconnect_port(reason=f"Mất kết nối phần cứng {self.connected_port_name}")
            time.sleep(0.1)

    # --- ĐIỀU KHIỂN ---
    def _toggle_direction(self):
        new_dir = not self._get_current_direction()
        if self.selected_motor_id == 0:
            for m_id in range(1, 5):
                self.motor_directions[m_id] = new_dir
        else:
            self.motor_directions[self.selected_motor_id] = new_dir
        self._update_ui_state()

    def _emergency_stop(self):
        for m_id in range(1, 5):
            self.motor_throttles[m_id] = 0.0
        self._update_ui_state()
        self._add_log("[EMERGENCY STOP] Toàn bộ 4 động cơ đã ngắt hoàn toàn!")

    # --- CANVAS DRAWING ---
    def _draw_gauge(self):
        self.gauge_canvas.delete("all")
        cx, cy, r = 110, 90, 65

        self.gauge_canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=-30, extent=240, style="arc", outline="#1E2330", width=10)

        th = self._get_current_throttle()
        extent = -(th / 100.0) * 240
        if extent != 0:
            self.gauge_canvas.create_arc(cx - r, cy - r, cx + r, cy + r, start=210, extent=extent, style="arc", outline="#00E5FF", width=10)

        raw_pwm = int((th / 100.0) * 255)
        m_str = "ALL" if self.selected_motor_id == 0 else f"M{self.selected_motor_id}"
        self.gauge_canvas.create_text(cx, cy - 8, text=f"{int(th)}%", fill="#FFFFFF", font=("monospace", 24, "bold"))
        self.gauge_canvas.create_text(cx, cy + 24, text=f"[{m_str}] PWM: {raw_pwm}", fill="#00E5FF", font=("monospace", 10, "bold"))

    def _draw_throttle(self):
        self.throttle_canvas.delete("all")
        w, h = 160, 280
        track_x = w // 2
        track_y1, track_y2 = 25, h - 25
        track_h = track_y2 - track_y1

        self.throttle_canvas.create_rectangle(track_x - 6, track_y1, track_x + 6, track_y2, fill="#090A0F", outline="#1E2330", width=2)

        th = self._get_current_throttle()
        handle_y = track_y2 - (th / 100.0) * track_h
        self.throttle_canvas.create_rectangle(track_x - 2, handle_y, track_x + 2, track_y2, fill="#00E5FF", outline="")

        hw, hh = 80, 34
        self.throttle_canvas.create_rectangle(
            track_x - hw // 2, handle_y - hh // 2, track_x + hw // 2, handle_y + hh // 2, fill="#252A36", outline="#00E5FF", width=2
        )

        for offset in [-15, 0, 15]:
            color = "#00E5FF" if offset == 0 else "#8A8F9E"
            self.throttle_canvas.create_line(track_x + offset, handle_y - 8, track_x + offset, handle_y + 8, fill=color, width=2)

        self.throttle_val_label.configure(text=f"{int(th)}%")

    def _on_throttle_drag(self, event):
        track_y1, track_y2 = 25, 255
        track_h = track_y2 - track_y1

        y = min(max(event.y, track_y1), track_y2)
        val = (1.0 - ((y - track_y1) / track_h)) * 100.0

        if self.selected_motor_id == 0:
            for m_id in range(1, 5):
                self.motor_throttles[m_id] = val
        else:
            self.motor_throttles[self.selected_motor_id] = val

        self._update_ui_state()

    # --- TRANSMIT & RECEIVE LOGIC ---
    def _transmit_loop(self):
        """
        Cấu trúc gói tin UART 10 Bytes gửi xuống ESP32 / STM32:
        [0xAA] [M1_PWM] [M2_PWM] [M3_PWM] [M4_PWM] [DIR_FLAGS] [CHECKSUM] [0x55]
        - DIR_FLAGS: Bit 0=M1, Bit 1=M2, Bit 2=M3, Bit 3=M4 (0=FWD, 1=REV)
        """
        while self.running:
            if self.is_connected and self.serial_port:
                try:
                    pwms = [int((self.motor_throttles[i] / 100.0) * 255) for i in range(1, 5)]
                    pwms = [max(0, min(255, p)) for p in pwms]

                    dir_flags = 0
                    for i in range(1, 5):
                        if self.motor_directions[i]:
                            dir_flags |= (1 << (i - 1))

                    checksum = (sum(pwms) + dir_flags) & 0xFF
                    packet = bytes([0xAA] + pwms + [dir_flags, checksum, 0x55])

                    self.serial_port.write(packet)

                    current_time = time.time()
                    if current_time - self.last_sent_time > 0.25:
                        hex_str = " ".join([f"0x{b:02X}" for b in packet])
                        self._add_log(f"[TX] {hex_str}")
                        self.last_sent_time = current_time

                except (serial.SerialException, OSError) as e:
                    self._disconnect_port(reason=f"Ghi dữ liệu thất bại ({e})")

            time.sleep(0.05)  # 20Hz

    def _receive_loop(self):
        while self.running:
            if self.is_connected and self.serial_port:
                try:
                    if self.serial_port.in_waiting > 0:
                        incoming = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                        if incoming:
                            self._add_log(f"[RX] {incoming}")
                except Exception:
                    pass
            time.sleep(0.02)

    def _add_log(self, text):
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_textbox.insert("end", f"[{now}] {text}\n")
        self.log_textbox.see("end")

    def _clear_log(self):
        self.log_textbox.delete("1.0", "end")

    def _on_close(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.destroy()


if __name__ == "__main__":
    app = MultiMotorCockpitApp()
    app.mainloop()