# -*- coding: utf-8 -*-
"""
# =============================================================================
# BIAXIAL STRETCHER CONTROL INTERFACE (v9.5.4)
# =============================================================================
# DESCRIPTION:
# This program provides a Graphical User Interface (GUI) to control a custom 
# Biaxial Stretcher powered by a Duet 2 WiFi/Ethernet controller. It allows 
# researchers to precisely manipulate X and Y axes simultaneously to stretch 
# materials, while monitoring temperature and position in real-time.
#
# ARCHITECTURE (Multiprocessing):
# To ensure the interface never "freezes" during hardware communication, this 
# script splits into two independent processes:
#   1. The GUI Process: Handles the window, buttons, and user input (PyQt6).
#   2. The Worker Process: A background "Postman" that handles all Serial 
#      communication (pyserial) and G-Code processing.
#
# KEY FEATURES:
# - Real-time Data Parsing: Uses Regular Expressions (Regex) to "pluck" 
#   coordinates and temperatures out of the hardware's text stream.
# - SD-Macro Mode: Wraps G-code in M28/M29 commands. This allows the 
#   PanelDue (onboard screen) to treat movements as "Print Jobs" so they 
#   can be paused or cancelled manually.
# - Console Filtering: Automatically hides repetitive "status spam" (like 'ok' 
#   messages) to keep the log clear for important WiFi and Error data.
# - Automated Calibration: Includes a math engine to update motor steps-per-mm 
#   (M92) based on user-measured physical displacement.
#
# TARGET HARDWARE: Duet 2 (RepRapFirmware) via USB Serial.
# AUTHORS: urosc + Gemini (AI Collaborator)
# DATE: March 2026
# =============================================================================
"""
# -*- coding: utf-8 -*-
import sys, serial, serial.tools.list_ports, time, multiprocessing, re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QHBoxLayout, QWidget, QDoubleSpinBox, QLabel, 
                             QComboBox, QGroupBox, QGridLayout, QTextEdit, 
                             QSlider, QLineEdit, QCheckBox, QMessageBox)
from PyQt6.QtCore import Qt, QTimer


# =============================================================================
# Helper class for the GUI
# =============================================================================
class BioSpeedBox(QDoubleSpinBox):
    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Up:
            self.setValue(self.value() + 0.5)
        elif event.key() == Qt.Key.Key_Down:
            self.setValue(self.value() - 0.5)
        elif event.key() == Qt.Key.Key_Left:
            self.setValue(self.value() - 0.01)
        elif event.key() == Qt.Key.Key_Right:
            self.setValue(self.value() + 0.01)
        else:
            super().keyPressEvent(event)

# =============================================================================
# HARDWARE ENGINE (The Background Worker)
# =============================================================================
def duet_worker(cmd_queue, res_queue, stop_event):
    ser = None                                           # Placeholder for the Serial object
    while not stop_event.is_set():                       # Loop forever until program closes
        try:
            while not cmd_queue.empty():                 # Check if GUI sent a command
                action, data = cmd_queue.get_nowait()    # Unpack command (e.g., "GCODE", "M112")
                if action == "CONNECT":                  # Logic to open the USB port
                    try:
                        if ser and ser.is_open: ser.close() # Reset port if already open
                        ser = serial.Serial(data, 115200, timeout=0.01) # Open at high speed
                        time.sleep(0.5)                  # Wait for hardware handshake
                        ser.write(b"M564 H0 S0\nM17\n")  # Setup: Ignore limits & enable motors
                        res_queue.put(("STATUS", "CONNECTED")) # Send success back to GUI
                    except Exception as e:
                        res_queue.put(("STATUS", f"OFFLINE: {e}")) # Send error if port fails
                elif action == "GCODE":                  # Logic for moving or heating
                    if ser and ser.is_open:
                        ser.write(f"{data}\n".encode('utf-8')) # Send text as "bytes" to Duet
                elif action == "ESTOP":                  # Logic for the Panic Button
                    if ser and ser.is_open:
                        ser.write(b"M112\n")             # M112 is the universal "Kill" command
                        res_queue.put(("STATUS", "!!! EMERGENCY RESET (M112) !!!"))
                        time.sleep(0.5); ser.close()     # Close port to prevent more commands
                elif action == "SHUTDOWN":               # Logic for closing the app
                    if ser and ser.is_open:
                        ser.write(b"M18\n")              # M18 releases motor holding current
                        time.sleep(0.2); ser.close()     # Safely disconnect
                    return                               # Exit this function/thread

            if ser and ser.is_open:                      # If connected, ask for status updates
                try:
                    while ser.in_waiting > 0:            # Check if Duet is talking back
                        line = ser.readline().decode('utf-8', errors='replace').strip()
                        if line: res_queue.put(("DATA", line)) # Pass hardware text to GUI
                    ser.write(b"M114\nM105\n")           # Poll: M114 (Pos) and M105 (Temp)
                    time.sleep(0.2)                      # Wait 200ms (5 updates per second)
                except: ser.close()                      # Close if wire is pulled out
        except Exception: pass                           # Prevent minor errors from crashing
        time.sleep(0.01)                                 # Keep CPU usage low

# =============================================================================
# MAIN GUI (The User Interface)
# =============================================================================
class StretcherGUI(QMainWindow):
    def __init__(self, cmd_queue, res_queue, stop_event):
        super().__init__()                               # Initialize the window
        self.cmd_queue = cmd_queue                       # Save the "Outbox" queue
        self.res_queue = res_queue                       # Save the "Inbox" queue
        self.stop_event = stop_event                     # Save the "Kill" switch
        self.total_target = 1.0                          # Default stretch distance (mm)

        self.setWindowTitle("Biaxial Stretcher")         # Set window title
        self.setFixedWidth(550)                          # Lock window width for neatness

        main_widget = QWidget()                          # Create a base container
        self.setCentralWidget(main_widget)               # Place container in window
        outer_layout = QVBoxLayout(main_widget)          # Top-to-bottom layout

        # --- READOUTS (Top Bar) ---
        disp_lyt = QHBoxLayout()                         # Left-to-right row for labels
        self.pos_display = QLabel("X: 0.00 | Y: 0.00")   # Create Position label
        self.pos_display.setStyleSheet("font-size: 16px; color: #1B5E20; font-family: 'Consolas'; background: #E8F5E9; padding: 5px; border: 1px solid #C8E6C9;")
        self.temp_display = QLabel("Temp: --")           # Create Temperature label
        self.temp_display.setStyleSheet("font-size: 16px; color: #B71C1C; font-family: 'Consolas'; background: #FFEBEE; padding: 5px; border: 1px solid #FFCDD2;")
        disp_lyt.addWidget(self.pos_display)             # Add Pos to the row
        disp_lyt.addWidget(self.temp_display)            # Add Temp to the row
        outer_layout.addLayout(disp_lyt)                 # Add the row to the top of window

        # --- SAFETY WARNING ---
        self.instr_label = QLabel("⚠️ PANEL DUE STOP: Press [STOP] in TOP-LEFT corner.")
        self.instr_label.setStyleSheet("color: #D84315; font-weight: bold; background: #FFF3E0; padding: 6px; border: 1px solid #FFE0B2; font-size: 10px;")
        self.instr_label.setAlignment(Qt.AlignmentFlag.AlignCenter) # Center text
        outer_layout.addWidget(self.instr_label)

        self.sd_mode_toggle = QCheckBox("SD-Macro Mode (Enables PanelDue Pause/Cancel)")
        self.sd_mode_toggle.setStyleSheet("font-size: 10px; color: #666;")
        outer_layout.addWidget(self.sd_mode_toggle)      # Add checkbox for advanced mode

        body_lyt = QHBoxLayout()                         # Create area for the two main columns
        left_col, right_col = QVBoxLayout(), QVBoxLayout() # Define left and right columns

        # GROUP 1: SETUP
        hw_group = QGroupBox("1. Setup")                 # Create titled box
        hl = QVBoxLayout()                               # Vertical layout for box items
        self.port_selector = QComboBox()                 # Dropdown for COM ports
        self.port_selector.addItems([p.device for p in serial.tools.list_ports.comports()])
        btn_conn = QPushButton("CONNECT")                # Connection button
        btn_conn.clicked.connect(lambda: self.cmd_queue.put(("CONNECT", self.port_selector.currentText())))
        self.ssid = QLineEdit(); self.ssid.setPlaceholderText("SSID") # WiFi Name box
        self.pw = QLineEdit(); self.pw.setPlaceholderText("Pass")     # WiFi Password box
        self.pw.setEchoMode(QLineEdit.EchoMode.Password) # Hide password with dots
        btn_save_wifi = QPushButton("SAVE WIFI")         # Save WiFi button
        btn_save_wifi.clicked.connect(self.save_wifi)    # Link to function
        
        wifi_ctrl_lyt = QHBoxLayout()                    # Row for WiFi Power/Reset
        self.btn_wifi_toggle = QPushButton("WIFI: OFF")  # Power toggle
        self.btn_wifi_toggle.setCheckable(True)           # Makes it a toggle (On/Off)
        self.btn_wifi_toggle.clicked.connect(self.toggle_wifi_power)
        btn_wifi_reset = QPushButton("RESET")            # WiFi Wipe button
        btn_wifi_reset.clicked.connect(self.reset_wifi_module)
        wifi_ctrl_lyt.addWidget(self.btn_wifi_toggle); wifi_ctrl_lyt.addWidget(btn_wifi_reset)
        
        # Add all Setup items into the vertical box layout
        hl.addWidget(self.port_selector); hl.addWidget(btn_conn); hl.addWidget(self.ssid); hl.addWidget(self.pw); hl.addWidget(btn_save_wifi); hl.addLayout(wifi_ctrl_lyt)
        hw_group.setLayout(hl); left_col.addWidget(hw_group) # Put Setup box in left column

        # GROUP 2: HEAT
        heat_group = QGroupBox("2. Heat")
        htl = QVBoxLayout()
        self.t_set = QDoubleSpinBox()                    # Number input for temp
        self.t_set.setRange(0, 50); self.t_set.setValue(37.0); self.t_set.setSuffix("°C")
        btn_h_on = QPushButton("SET"); btn_h_off = QPushButton("OFF")
        btn_h_on.clicked.connect(lambda: self.cmd_queue.put(("GCODE", f"M140 H0 S{self.t_set.value()}")))
        btn_h_off.clicked.connect(lambda: self.cmd_queue.put(("GCODE", "M140 H-1"))) # H-1 disables heater
        htl.addWidget(self.t_set); htl.addWidget(btn_h_on); htl.addWidget(btn_h_off)
        heat_group.setLayout(htl); left_col.addWidget(heat_group)

        # GROUP 3: MOTION
        move_group = QGroupBox("3. Motion")
        ml = QVBoxLayout()
        self.dist_label = QLabel("Stroke: 1.00mm")
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(1, 500); self.slider.setValue(100)
        self.slider.valueChanged.connect(self.update_dist)
        
        # Use the custom BioSpeedBox class
        self.speed_box = BioSpeedBox()
        self.speed_box.setRange(0.001, 50.0) 
        self.speed_box.setDecimals(3)      # Important for 0.01 precision
        self.speed_box.setValue(10.0)      # Default starting speed
        self.speed_box.setSingleStep(0.5)  # The clickable arrows jump by 0.5
        self.speed_box.setSuffix(" mm/min")
        self.speed_box.setToolTip("↑/↓: ±0.5 | ←/→: ±0.01") # Hint for the user
        
        ml.addWidget(self.dist_label)
        ml.addWidget(self.slider)
        ml.addWidget(self.speed_box)

        jg = QGridLayout()                               # 2x2 grid for manual jog buttons
        jog_btns = [("▶▶ Y ◀◀", 0, 0, "Y"), ("◀◀ Y ▶▶", 0, 1, "Y-"), 
                    ("▶▶ X ◀◀", 1, 0, "X"), ("◀◀ X ▶▶", 1, 1, "X-")]
        for t, r, c, ax in jog_btns:
            b = QPushButton(t); b.setFixedSize(110, 35) 
            b.clicked.connect(lambda ch, a=ax: self.move_sd(a, 1)) # Connect axis to click
            jg.addWidget(b, r, c)
        ml.addLayout(jg)
        
        sl = QHBoxLayout(); bs = QPushButton("STRETCH"); br = QPushButton("RETRACT")
        bs.clicked.connect(lambda: self.move_sd("BOTH", -1)); br.clicked.connect(lambda: self.move_sd("BOTH", 1))
        sl.addWidget(bs); sl.addWidget(br); ml.addLayout(sl)
        move_group.setLayout(ml); right_col.addWidget(move_group)

        # GROUP 4: CALIBRATE
        cal_group = QGroupBox("4. Cal")
        cl = QHBoxLayout()
        self.meas_in = QDoubleSpinBox()                  # Input for measured distance
        self.meas_in.setRange(0.01, 500); self.meas_in.setValue(1.0)
        cx=QPushButton("X"); cy=QPushButton("Y"); btn_help = QPushButton("?"); btn_help.setFixedWidth(25)
        cx.clicked.connect(lambda: self.run_calibration("X")); cy.clicked.connect(lambda: self.run_calibration("Y"))
        btn_help.clicked.connect(self.show_cal_help)     # Instruction popup
        cl.addWidget(self.meas_in); cl.addWidget(cx); cl.addWidget(cy); cl.addWidget(btn_help)
        cal_group.setLayout(cl); right_col.addWidget(cal_group)

        body_lyt.addLayout(left_col, 1); body_lyt.addLayout(right_col, 1) # Assemble columns
        outer_layout.addLayout(body_lyt)                 # Add columns to main window

        self.btn_estop = QPushButton("HARD EMERGENCY STOP (M112)")
        self.btn_estop.setStyleSheet("background: #B71C1C; color: white; font-weight: bold; height: 45px;")
        self.btn_estop.clicked.connect(lambda: self.cmd_queue.put(("ESTOP", "KILL")))
        outer_layout.addWidget(self.btn_estop)           # Add E-Stop to bottom

        self.console = QTextEdit()                       # Large black logging box
        self.console.setReadOnly(True); self.console.setMinimumHeight(120)
        self.console.setStyleSheet("background: #1e1e1e; color: #00ff00; font-family: 'Consolas'; font-size: 11px;")
        outer_layout.addWidget(self.console)

        self.timer = QTimer()                            # Create the update clock
        self.timer.timeout.connect(self.update_ui)       # Link clock to update function
        self.timer.start(100)                            # Run every 0.1 seconds

    # ---------------- LOGIC ----------------
    def toggle_wifi_power(self):
        st = "1" if self.btn_wifi_toggle.isChecked() else "0"
        self.cmd_queue.put(("GCODE", f"M552 S{st}"))      # M552 S1 turns on WiFi module
        self.btn_wifi_toggle.setText(f"WIFI: {'ON' if st == '1' else 'OFF'}")
        self.btn_wifi_toggle.setStyleSheet("background-color: #C8E6C9;" if st == "1" else "")

    def reset_wifi_module(self):
        self.cmd_queue.put(("GCODE", "M552 S-1\nM588 S\"*\"\nG4 P500\nM552 S0")) # Wipe all saved WiFis

    def show_cal_help(self):
        QMessageBox.information(self, "Calibration", "1. Jog known dist.\n2. Measure actual.\n3. Input measured.\n4. Click X or Y.")

    def save_wifi(self):
        self.cmd_queue.put(("GCODE", f"M552 S0\nM587 S\"{self.ssid.text()}\" P\"{self.pw.text()}\"\nM552 S1"))

    def move_sd(self, axis, direction):
        speed = self.speed_box.value()
        dist = (self.total_target / 2.0) * direction      # Center-out movement math
        g_move = f"G91\nG1 X{dist} Y{dist} F{speed/2.0}\nG90" if axis == "BOTH" else f"G91\nG1 {axis[0]}{dist if '-' not in axis else -self.total_target} F{speed}\nG90"
        if self.sd_mode_toggle.isChecked():               # Logic to make movement "pausable"
            full_cmd = f"M28 0:/sys/active.g\n{g_move}\nM29\nM32 0:/sys/active.g" # Write to SD and run
        else:
            full_cmd = g_move                             # Send directly via USB
        self.cmd_queue.put(("GCODE", full_cmd))

    def update_dist(self):
        self.total_target = self.slider.value()/100.0     # Convert slider (100) to mm (1.00)
        self.dist_label.setText(f"Stroke: {self.total_target:.2f}mm")

    def run_calibration(self, ax):
        base = 100.0 if ax == "X" else 120.0              # Assumed motor steps/mm
        new_val = (self.total_target / self.meas_in.value()) * base
        self.cmd_queue.put(("GCODE", f"M92 {ax}{new_val:.2f}")) # Update steps per mm

    def update_ui(self):
        while not self.res_queue.empty():                # Process all hardware messages in queue
            rtype, rdata = self.res_queue.get_nowait()
            if rtype == "DATA":
                m_pos = re.search(r"X:\s*([-+]?\d*\.\d+|\d+)", rdata) # Find X position using Regex
                if m_pos:
                    m_y = re.search(r"Y:\s*([-+]?\d*\.\d+|\d+)", rdata) # Find Y position
                    self.pos_display.setText(f"X: {m_pos.group(1)} | Y: {m_y.group(1) if m_y else '0.00'}")
                m_temp = re.search(r"B:\s*([-+]?\d*\.\d+|\d+)", rdata) # Find Bed Temp using Regex
                if m_temp: self.temp_display.setText(f"Temp: {m_temp.group(1)}°C")
                # SPAM FILTER: Only print to console if it's NOT coordinates or "ok"
                is_spam = any(x in rdata for x in ["ok", "X:", "Y:", "Z:", "T:0", "B:"])
                if not is_spam: self.console.append(rdata) # Add line to log box
            elif rtype == "STATUS": self.console.append(f"<b>>>> {rdata}</b>") # Bold status updates
            self.console.verticalScrollBar().setValue(self.console.verticalScrollBar().maximum()) # Auto-scroll

    def closeEvent(self, event):                         # Runs when user clicks [X]
        self.cmd_queue.put(("SHUTDOWN", "KILL"))         # Disable motors
        self.stop_event.set()                            # Stop the worker thread
        event.accept()                                   # Close application

if __name__ == "__main__":
    multiprocessing.freeze_support()                     # Support for .exe bundling
    q1, q2, ev = multiprocessing.Queue(), multiprocessing.Queue(), multiprocessing.Event()
    multiprocessing.Process(target=duet_worker, args=(q1, q2, ev), daemon=True).start() # Start hardware process
    app = QApplication(sys.argv); gui = StretcherGUI(q1, q2, ev); gui.show(); sys.exit(app.exec()) # Start GUI