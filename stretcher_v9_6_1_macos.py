# -*- coding: utf-8 -*-
"""
# =============================================================================
# BIAXIAL STRETCHER CONTROL INTERFACE (v9.6.1 - macOS compatible)
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
import sys
import time
import re
import math
import queue
import multiprocessing

import serial
import serial.tools.list_ports
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QHBoxLayout, QWidget, QDoubleSpinBox, QLabel, 
                             QComboBox, QGroupBox, QGridLayout, QTextEdit, 
                             QSlider, QLineEdit, QCheckBox, QMessageBox, QProgressBar)
from PyQt6.QtCore import Qt, QTimer


# RepRapFirmware 3.4.1 absolute minimum vector feed rate.
RRF_MIN_VECTOR_FEEDRATE = 0.60  # mm/min
DIRECT_MIN_SEPARATION_RATE = 2.0 * RRF_MIN_VECTOR_FEEDRATE


# =============================================================================
# Helper class for the GUI
# =============================================================================
class BioSpeedBox(QDoubleSpinBox):
    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Up:
            self.setValue(self.value() + 1)
        elif event.key() == Qt.Key.Key_Down:
            self.setValue(self.value() - 1)
        elif event.key() == Qt.Key.Key_Left:
            self.setValue(self.value() - 0.5)
        elif event.key() == Qt.Key.Key_Right:
            self.setValue(self.value() + 0.5)
        else:
            super().keyPressEvent(event)

# =============================================================================
# HARDWARE ENGINE (The Background Worker)
# =============================================================================
def duet_worker(cmd_queue, res_queue, stop_event):
    ser = None                                           # Placeholder for the Serial object
    while not stop_event.is_set():                       # Loop forever until program closes
        try:
            while True:                                  # Drain all queued GUI commands
                try:
                    action, data = cmd_queue.get_nowait()
                except queue.Empty:
                    break
                if action == "CONNECT":                  # Logic to open the USB port
                    try:
                        if ser and ser.is_open: ser.close() # Reset port if already open
                        ser = serial.Serial(data, 115200, timeout=0.01) # Open at high speed
                        time.sleep(0.5)                  # Wait for hardware handshake
                        ser.write(b"M552 S0\nM564 H0 S0\nM17\n") # Disable network (idle)
                        ser.write(b"M564 H0 S0\nM17\n")  # Setup: Ignore limits & enable motors
                        res_queue.put(("STATUS", "CONNECTED")) # Send success back to GUI
                    except Exception as e:
                        res_queue.put(("STATUS", f"OFFLINE: {e}")) # Send error if port fails
                elif action == "GCODE":                  # Logic for moving or heating
                    if ser and ser.is_open:
                        ser.write(f"{data}\n".encode('utf-8')) # Send text as "bytes" to Duet
                elif action == "SD_JOB":                 # Upload and start a G-code job safely
                    if ser and ser.is_open:
                        path, content = data
                        try:
                            # Keep status polling out of the M28/M29 upload window.
                            ser.write(f"M28 {path}\n".encode('utf-8'))
                            ser.flush()
                            time.sleep(0.10)
                            for line in content.splitlines():
                                ser.write(f"{line}\n".encode('utf-8'))
                                ser.flush()
                                time.sleep(0.005)
                            ser.write(b"M29\n")
                            ser.flush()
                            time.sleep(0.30)
                            ser.write(f"M32 {path}\n".encode('utf-8'))
                            ser.flush()
                        except Exception as e:
                            res_queue.put(("STATUS", f"SD UPLOAD ERROR: {e}"))
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
        self.total_target = 1.0                          # Default separation change (mm)
        self.stepped_protocol_active = False
        self.protocol_total_cycles = 0
        self.motion_active = False

        self.setWindowTitle("Biaxial Stretcher")         # Set window title
        self.setFixedWidth(550)                          # Lock window width for neatness

        main_widget = QWidget()                          # Create a base container
        self.setCentralWidget(main_widget)               # Place container in window
        outer_layout = QVBoxLayout(main_widget)          # Top-to-bottom layout
        
        # Reset timer
        self.progress_reset_timer = QTimer(self)
        self.progress_reset_timer.setSingleShot(True)
        self.progress_reset_timer.timeout.connect(self.reset_progress_ui)
        
        # --- READOUTS (Top Bar) ---
        disp_lyt = QHBoxLayout()                         # Left-to-right row for labels
        self.pos_display = QLabel("X: 0.00 | Y: 0.00")   # Create Position label
        self.pos_display.setStyleSheet("font-size: 16px; color: #1B5E20; font-family: 'Menlo', 'Monaco', 'Consolas', monospace; background: #E8F5E9; padding: 5px; border: 1px solid #C8E6C9;")
        self.temp_display = QLabel("Temp: --")           # Create Temperature label
        self.temp_display.setStyleSheet("font-size: 16px; color: #B71C1C; font-family: 'Menlo', 'Monaco', 'Consolas', monospace; background: #FFEBEE; padding: 5px; border: 1px solid #FFCDD2;")
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
        self.port_selector = QComboBox()                 # Dropdown for serial ports

        port_row = QHBoxLayout()
        port_row.addWidget(self.port_selector)

        btn_refresh_ports = QPushButton("Refresh")
        btn_refresh_ports.setFixedWidth(70)
        btn_refresh_ports.setToolTip("Refresh available USB serial ports")
        btn_refresh_ports.clicked.connect(self.refresh_ports)
        port_row.addWidget(btn_refresh_ports)

        self.btn_conn = QPushButton("CONNECT")
        self.btn_conn.clicked.connect(self.connect_selected_port)

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
        hl.addLayout(port_row)
        hl.addWidget(self.btn_conn)
        hl.addWidget(self.ssid)
        hl.addWidget(self.pw)
        hl.addWidget(btn_save_wifi)
        hl.addLayout(wifi_ctrl_lyt)
        hw_group.setLayout(hl); left_col.addWidget(hw_group) # Put Setup box in left column

        self.refresh_ports()

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
        self.slider.setRange(1, 1000); self.slider.setValue(100)
        self.slider.valueChanged.connect(self.update_dist)
               
        # Continuous-motion speed. The GUI value is separation speed, while
        # the motor receives half of that value. Therefore 1.20 mm/min is
        # the lowest GUI rate that keeps a single-axis G1 at RRF's 0.60 mm/min floor.
        self.speed_box = BioSpeedBox()
        self.speed_box.setRange(DIRECT_MIN_SEPARATION_RATE, 1000.0)
        self.speed_box.setDecimals(2)
        self.speed_box.setValue(10.0)
        self.speed_box.setSingleStep(0.10)
        self.speed_box.setSuffix(" mm/min")
        self.speed_box.setToolTip(
            "Continuous separation rate. Minimum 1.20 mm/min because the motor rate is halved."
        )

        # Timed stepped mode is used for average rates below the firmware's
        # continuous-motion floor. It always runs as an SD-card job.
        self.timed_mode_toggle = QCheckBox("Timed stepped stretch (SD only)")
        self.timed_mode_toggle.setToolTip(
            "Distributes whole motor microsteps across a selected duration."
        )
        self.duration_box = QDoubleSpinBox()
        self.duration_box.setRange(1.0, 1440.0)
        self.duration_box.setDecimals(1)
        self.duration_box.setSingleStep(5.0)
        self.duration_box.setValue(60.0)
        self.duration_box.setSuffix(" min")
        self.duration_box.setEnabled(False)

        self.avg_rate_label = QLabel("Average rate: --")
        self.avg_rate_label.setStyleSheet("font-size: 10px; color: #555;")

        self.timed_mode_toggle.toggled.connect(self.on_timed_mode_toggled)
        self.duration_box.valueChanged.connect(self.update_protocol_summary)
       
        ml.addWidget(self.dist_label)
        ml.addWidget(self.slider)
        ml.addWidget(self.speed_box)
        ml.addWidget(self.timed_mode_toggle)
        ml.addWidget(self.duration_box)
        ml.addWidget(self.avg_rate_label)

        jg = QGridLayout()                               # 2x2 grid for manual jog buttons
        self.jog_buttons = []
        jog_btns = [("▶▶ Y ◀◀", 0, 0, "Y"), ("◀◀ Y ▶▶", 0, 1, "Y-"), 
                    ("▶▶ X ◀◀", 1, 0, "X"), ("◀◀ X ▶▶", 1, 1, "X-")]
        # Pass 1.0 for positive jog, -1.0 for negative jog
        for t, r, c, ax in jog_btns:
            b = QPushButton(t)
            b.setFixedSize(110, 35) 
            clean_ax = ax[0]   # 1. Strip the '-' so 'axis' is always just "X" or "Y"
            # 2. Assign the direction based on the original string
            d_val = -1.0 if '-' in ax else 1.0 # (If it has a '-', it's -1.0, otherwise 1.0)
            # 3. Connect using dist=1.0. 
            b.clicked.connect(lambda ch, a=clean_ax, d=d_val: self.move_sd(a, d))
            self.jog_buttons.append(b)
            jg.addWidget(b, r, c)          
        ml.addLayout(jg)
        
        sl = QHBoxLayout()
        self.btn_stretch = QPushButton("STRETCH")
        self.btn_retract = QPushButton("RETRACT")
        self.btn_stretch.clicked.connect(
            lambda: self.move_sd("BOTH", -1)
        )
        self.btn_retract.clicked.connect(
            lambda: self.move_sd("BOTH", 1)
        )
        sl.addWidget(self.btn_stretch)
        sl.addWidget(self.btn_retract)
        ml.addLayout(sl)
        move_group.setLayout(ml); right_col.addWidget(move_group)
        
        # Create a dedicated "Soft Stop" button
        self.btn_soft_stop = QPushButton("STOP MOVEMENT (SOFT)")
        self.btn_soft_stop.setStyleSheet("""
            background-color: #FB8C00; 
            color: white; 
            font-weight: bold; 
            height: 35px; 
            border-radius: 5px;
        """)
        self.btn_soft_stop.clicked.connect(self.abort_movement)
        ml.addWidget(self.btn_soft_stop)
        
        # progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setStyleSheet("""
            QProgressBar { border: 1px solid #bbb; border-radius: 5px; text-align: center; height: 20px; }
            QProgressBar::chunk { background-color: #4CAF50; }
        """)
        outer_layout.addWidget(self.progress_bar)
        # Initialize timing variables
        self.move_timer = QTimer()
        self.move_timer.timeout.connect(self.advance_progress)
        self.start_time = 0
        self.expected_duration = 0

        # GROUP 4: CALIBRATE
        cal_group = QGroupBox("4. Cal")
        cl = QHBoxLayout()
        self.meas_in = QDoubleSpinBox()                  # Input for measured distance
        self.meas_in.setRange(0.01, 1000); self.meas_in.setValue(1.0)
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
        self.console.setStyleSheet("background: #1e1e1e; color: #00ff00; font-family: 'Menlo', 'Monaco', 'Consolas', monospace; font-size: 11px;")
        outer_layout.addWidget(self.console)

        self.timer = QTimer()                            # Create the update clock
        self.timer.timeout.connect(self.update_ui)       # Link clock to update function
        self.timer.start(100)                            # Run every 0.1 seconds

    # ---------------- LOGIC ----------------
    def refresh_ports(self):
        """Refresh and display available serial ports."""
        previous_port = self.port_selector.currentData()
        self.port_selector.clear()

        ports = list(serial.tools.list_ports.comports())

        # pySerial does not guarantee port order. Put probable USB ports first.
        ports.sort(
            key=lambda port: (
                0 if "usb" in port.device.lower() else 1,
                port.device.lower(),
            )
        )

        for port in ports:
            description = port.description or ""
            if description and description.lower() != "n/a":
                label = f"{port.device} — {description}"
            else:
                label = port.device

            # Display a useful label while storing the real device path.
            self.port_selector.addItem(label, port.device)

        self.btn_conn.setEnabled(bool(ports))

        if previous_port:
            previous_index = self.port_selector.findData(previous_port)
            if previous_index >= 0:
                self.port_selector.setCurrentIndex(previous_index)

    def connect_selected_port(self):
        """Connect using the actual serial-device path stored in the selector."""
        port = self.port_selector.currentData()

        if not port:
            QMessageBox.warning(
                self,
                "Serial connection",
                "No serial port is available. Connect the Duet by USB and press Refresh.",
            )
            return

        self.cmd_queue.put(("CONNECT", port))

    def toggle_wifi_power(self):
        st = "1" if self.btn_wifi_toggle.isChecked() else "0"
        self.cmd_queue.put(("GCODE", f"M552 S{st}"))      # M552 S1 turns on WiFi module
        self.btn_wifi_toggle.setText(f"WIFI: {'ON' if st == '1' else 'OFF'}")
        self.btn_wifi_toggle.setStyleSheet("background-color: #C8E6C9;" if st == "1" else "")

    def reset_wifi_module(self):
        self.cmd_queue.put(("GCODE", "M552 S-1\nM588 S\"*\"\nG4 P500\nM552 S0")) # Wipe all saved WiFis

    def show_cal_help(self):
        msg = QMessageBox(self)
        msg.setWindowTitle("How to Calibrate Motors")
        msg.setIcon(QMessageBox.Icon.Information)
        
        # Detailed HTML-formatted text for better readability
        text = (
            "<h3>Biaxial Calibration Guide</h3>"
            "<p>If the stretcher moves 10mm on screen but 11mm in reality, follow these steps:</p>"
            "<ol>"
            "<li><b>Jog:</b> Move the axis a known distance (e.g., use the 'STRETCH' button to move 5mm).</li>"
            "<li><b>Measure:</b> Use a digital caliper to measure the <i>actual</i> physical displacement of the grips.</li>"
            "<li><b>Input:</b> Type that physical measurement into the 'Measured' box.</li>"
            "<li><b>Update:</b> Click the <b>[X]</b> or <b>[Y]</b> button.</li>"
            "</ol>"
            "<p><i>Note: The software will automatically calculate the new 'Steps-per-mm' (M92) and send it to the Duet controller.</i></p>"
        )
        
        msg.setText(text)
        msg.setStandardButtons(QMessageBox.StandardButton.Ok)
        msg.exec()
        
    def save_wifi(self):
        self.cmd_queue.put(("GCODE", f"M552 S0\nM587 S\"{self.ssid.text()}\" P\"{self.pw.text()}\"\nM552 S1"))

    def set_motion_active(self, active):
        """Lock motion settings while a controller job is active."""
        self.motion_active = active
        timed = self.timed_mode_toggle.isChecked()

        self.slider.setEnabled(not active)
        self.btn_stretch.setEnabled(not active)
        self.btn_retract.setEnabled(not active)
        self.timed_mode_toggle.setEnabled(not active)

        self.speed_box.setEnabled((not active) and (not timed))
        self.duration_box.setEnabled((not active) and timed)
        self.sd_mode_toggle.setEnabled((not active) and (not timed))

        for button in self.jog_buttons:
            button.setEnabled(not active)

    def on_timed_mode_toggled(self, checked):
        if checked:
            self.sd_mode_toggle.setChecked(True)
    
        timed_tip = (
            "Starts the selected timed stepped movement on this axis."
            if checked
            else
            "Moves this axis using the selected stroke and continuous rate."
        )
    
        for button in self.jog_buttons:
            button.setToolTip(timed_tip)
    
        self.set_motion_active(self.motion_active)
        self.update_protocol_summary()
        
    def update_protocol_summary(self):
        """Show the average separation rate selected for timed stepped mode."""
        if not self.timed_mode_toggle.isChecked():
            self.avg_rate_label.setText("Average rate: --")
            return
    
        duration = self.duration_box.value()
    
        if duration <= 0:
            self.avg_rate_label.setText("Average rate: invalid duration")
            return
    
        average_rate = self.total_target / duration
    
        self.avg_rate_label.setText(
            f"Average rate: {average_rate:.4f} mm/min "
            f"({self.total_target:.2f} mm in {duration:.1f} min."
        )

    def build_timed_protocol_gcode(
            self, target_dist, duration_minutes, direction, axis="BOTH"):
        '''Build an RRF 3.4-compatible stepped SD job.

        The job reads the current X/Y steps-per-mm from the firmware object model,
        rounds each enabled endpoint to whole configured microsteps, and distributes
        those steps evenly so X-only, Y-only, or X+Y movement ends at the selected
        duration.
        '''
        axis = str(axis).upper()
        if axis not in {"X", "Y", "BOTH"}:
            raise ValueError(f"Unsupported timed-protocol axis: {axis}")

        direction = -1 if direction < 0 else 1
        duration_ms = int(round(duration_minutes * 60_000.0))

        if axis in {"X", "BOTH"}:
            steps_x_line = (
                "var stepsX = "
                "{floor((var.targetSep * var.spmX / 2.0) + 0.5)}"
            )
        else:
            steps_x_line = "var stepsX = 0"

        if axis in {"Y", "BOTH"}:
            steps_y_line = (
                "var stepsY = "
                "{floor((var.targetSep * var.spmY / 2.0) + 0.5)}"
            )
        else:
            steps_y_line = "var stepsY = 0"

        if axis == "BOTH":
            max_path_line = (
                "var maxPath = "
                "{sqrt((var.xStepMm * var.xStepMm) + "
                "(var.yStepMm * var.yStepMm))}"
            )
        elif axis == "X":
            max_path_line = "var maxPath = var.xStepMm"
        else:
            max_path_line = "var maxPath = var.yStepMm"

        return f'''\
var targetSep = {target_dist:.6f}
var durationMs = {duration_ms}
var direction = {direction}
var selectedAxis = "{axis}"
var minFeed = {RRF_MIN_VECTOR_FEEDRATE:.3f}
var spmX = move.axes[0].stepsPerMm
var spmY = move.axes[1].stepsPerMm
{steps_x_line}
{steps_y_line}
var cycles = max(var.stepsX, var.stepsY)
if var.cycles < 1
  M118 P1 S"__STEP_PROTOCOL_ERROR__:Stroke below one configured step on selected axis" L0
  abort "Timed stroke is below one configured motor step on the selected axis"
var xStepMm = {{1.0 / var.spmX}}
var yStepMm = {{1.0 / var.spmY}}
var cycleMs = {{var.durationMs / var.cycles}}
{max_path_line}
var maxMoveMs = {{var.maxPath / var.minFeed * 60000.0}}
var prevX = 0
var prevY = 0
var nextX = 0
var nextY = 0
var dxSteps = 0
var dySteps = 0
var dxMm = 0.0
var dyMm = 0.0
var pathMm = 0.0
var moveMs = 0.0
var dwellMs = 0
var preMs = 0
var postMs = 0
if var.cycleMs <= var.maxMoveMs
  M118 P1 S"__STEP_PROTOCOL_ERROR__:Duration too short for minimum feed rate" L0
  abort "Timed duration is too short for the configured steps and minimum feed rate"
M118 P1 S{{"__STEP_PROTOCOL_START__:"^var.cycles}} L0
M118 P1 S{{"Timed protocol axis="^var.selectedAxis^", steps X="^var.stepsX^", Y="^var.stepsY}} L0
while iterations < var.cycles
  set var.nextX = {{floor((((iterations + 1) * var.stepsX) / var.cycles) + 0.5)}}
  set var.nextY = {{floor((((iterations + 1) * var.stepsY) / var.cycles) + 0.5)}}
  set var.dxSteps = var.nextX - var.prevX
  set var.dySteps = var.nextY - var.prevY
  set var.dxMm = {{var.dxSteps / var.spmX}}
  set var.dyMm = {{var.dySteps / var.spmY}}
  set var.pathMm = {{sqrt((var.dxMm * var.dxMm) + (var.dyMm * var.dyMm))}}
  set var.moveMs = {{var.pathMm / var.minFeed * 60000.0}}
  set var.dwellMs = max(0, floor(var.cycleMs - var.moveMs))
  set var.preMs = floor(var.dwellMs / 2.0)
  set var.postMs = var.dwellMs - var.preMs
  G4 P{{var.preMs}}
  G91
  if var.dxSteps > 0 && var.dySteps > 0
    G1 X{{var.direction * var.dxMm}} Y{{var.direction * var.dyMm}} F{{var.minFeed}}
  elif var.dxSteps > 0
    G1 X{{var.direction * var.dxMm}} F{{var.minFeed}}
  else
    G1 Y{{var.direction * var.dyMm}} F{{var.minFeed}}
  G90
  M400
  G4 P{{var.postMs}}
  M118 P1 S{{"__STEP_PROGRESS__:"^(iterations + 1)^"/"^var.cycles}} L0
  set var.prevX = var.nextX
  set var.prevY = var.nextY
  M400
M118 P1 S"__MOVE_DONE__" L0'''

    def start_timed_protocol(self, axis, direction):
        '''Write and start a low-average-rate stepped protocol on the SD card.'''
        axis = str(axis).upper()
        if axis not in {"X", "Y", "BOTH"}:
            QMessageBox.warning(
                self,
                "Timed stepped mode",
                f"Unsupported axis selection: {axis}"
            )
            return

        # This mode is intentionally SD-only so PanelDue pause/cancel remains available.
        self.sd_mode_toggle.setChecked(True)
        self.progress_reset_timer.stop()
        self.move_timer.stop()
        self.stepped_protocol_active = True
        self.protocol_total_cycles = 0
        self.set_motion_active(True)

        duration_minutes = self.duration_box.value()
        self.expected_duration = duration_minutes * 60.0 + 0.5
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Writing timed SD job...")

        job = self.build_timed_protocol_gcode(
            self.total_target,
            duration_minutes,
            direction,
            axis
        )

        job_content = "G4 P500\n" + job
        axis_filename = {"BOTH": "xy", "X": "x", "Y": "y"}[axis]
        self.cmd_queue.put((
            "SD_JOB",
            (f"0:/gcodes/timed_stretch_{axis_filename}.g", job_content)
        ))

    def move_sd(self, axis, direction):
        # Prevent the previous movement's delayed reset from resetting this move.
        self.progress_reset_timer.stop()

        if self.timed_mode_toggle.isChecked():
            self.start_timed_protocol(axis, direction)
            return

        sd_mode = self.sd_mode_toggle.isChecked()
        target_dist = self.total_target

        # One motor mechanically changes the separation between opposing grippers.
        motor_move = (target_dist / 2.0) * direction

        # GUI speed is separation speed, so the motor receives half that speed.
        motor_feedrate = self.speed_box.value() / 2.0

        if motor_feedrate < RRF_MIN_VECTOR_FEEDRATE:
            QMessageBox.warning(
                self,
                "Rate below firmware minimum",
                "Continuous movement requires at least 1.20 mm/min separation rate. "
                "Use Timed stepped stretch for lower average rates."
            )
            return

        self.expected_duration = (abs(motor_move) / motor_feedrate) * 60.0
        if sd_mode:
            self.expected_duration += 0.5

        self.set_motion_active(True)
        self.start_time = time.monotonic()
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Starting...")
        self.move_timer.start(100)

        if axis == "BOTH":
            # F is vector speed; sqrt(2) keeps each X/Y motor at motor_feedrate.
            xy_feedrate = motor_feedrate * math.sqrt(2.0)
            g_move = (
                "G91\n"
                f"G1 X{motor_move:.6f} Y{motor_move:.6f} F{xy_feedrate:.3f}\n"
                "G90"
            )
        else:
            g_move = (
                "G91\n"
                f"G1 {axis}{motor_move:.6f} F{motor_feedrate:.3f}\n"
                "G90"
            )

        move_sequence = (
            f"{g_move}\n"
            "M400\n"
            'M118 P1 S"__MOVE_DONE__" L0'
        )

        if sd_mode:
            job_content = "G4 P500\n" + move_sequence
            self.cmd_queue.put((
                "SD_JOB",
                ("0:/gcodes/active.g", job_content)
            ))
        else:
            self.cmd_queue.put(("GCODE", move_sequence))

    def abort_movement(self):
        """ Stops the G-code file and keeps motors energized. """
        # M0 stops the 'SD Print' initiated by your SD-Macro Mode
        self.cmd_queue.put(("GCODE", "M0"))
        
        # Sync the GUI
        self.move_timer.stop()
        self.progress_reset_timer.stop()
        self.expected_duration = 0
        self.stepped_protocol_active = False
        self.protocol_total_cycles = 0
        self.set_motion_active(False)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("ABORTED - MOTORS HOLDING")
        self.console.append("<b style='color: #FB8C00;'>>>> MOVEMENT ABORTED (M0)</b>")
        
    def update_dist(self):
        self.total_target = self.slider.value()/100.0     # Slider 100 -> 1.00 mm separation
        self.dist_label.setText(f"Stroke: {self.total_target:.2f}mm")
        self.update_protocol_summary()

    def run_calibration(self, ax):
        """Scale the current live M92 value instead of using hard-coded defaults."""
        measured = self.meas_in.value()
        if measured <= 0:
            return

        axis_index = {"X": 0, "Y": 1}[ax]
        scale = self.total_target / measured
        command = (
            f"M92 {ax}{{move.axes[{axis_index}].stepsPerMm * {scale:.8f}}}\n"
            "M92"
        )
        self.cmd_queue.put(("GCODE", command))
        self.console.append(
            f"<b>CALIBRATION: scaling current {ax} steps/mm by {scale:.6f}. "
            "The following M92 response reports the new live value.</b>"
        )

    def update_ui(self):
        while True:                                      # Drain all worker responses
            try:
                rtype, rdata = self.res_queue.get_nowait()
            except queue.Empty:
                break
            if rtype == "DATA":
                protocol_error = re.search(r"__STEP_PROTOCOL_ERROR__:(.+)", rdata)
                if protocol_error:
                    self.move_timer.stop()
                    self.stepped_protocol_active = False
                    self.protocol_total_cycles = 0
                    self.set_motion_active(False)
                    self.progress_bar.setValue(0)
                    self.progress_bar.setFormat("Timed protocol error")
                    self.console.append(
                        f"<b style='color: #B71C1C;'>{protocol_error.group(1)}</b>"
                    )
                    continue

                protocol_start = re.search(r"__STEP_PROTOCOL_START__:(\d+)", rdata)
                if protocol_start:
                    self.protocol_total_cycles = int(protocol_start.group(1))
                    self.stepped_protocol_active = True
                    self.progress_bar.setValue(0)
                    self.progress_bar.setFormat(
                        f"Stepped stretch: 0/{self.protocol_total_cycles}"
                    )
                    continue

                protocol_progress = re.search(r"__STEP_PROGRESS__:(\d+)/(\d+)", rdata)
                if protocol_progress:
                    completed = int(protocol_progress.group(1))
                    total = int(protocol_progress.group(2))
                    self.protocol_total_cycles = total
                    percentage = round((completed / total) * 100) if total else 0
                    self.progress_bar.setValue(max(0, min(99, percentage)))
                    self.progress_bar.setFormat(
                        f"Stepped stretch: {completed}/{total}"
                    )
                    continue

                # Duet reports this only after M400 confirms movement completion.
                if "__MOVE_DONE__" in rdata:
                    self.finish_movement()
                    continue
            
                m_pos = re.search(r"X:\s*([-+]?\d*\.\d+|\d+)", rdata)
                if m_pos:
                    m_y = re.search(r"Y:\s*([-+]?\d*\.\d+|\d+)", rdata)
                    self.pos_display.setText(f"X: {m_pos.group(1)} | Y: {m_y.group(1) if m_y else '0.00'}")
                
                m_temp = re.search(r"B:\s*([-+]?\d*\.\d+|\d+)", rdata) # Find Bed Temp using Regex
                if m_temp: self.temp_display.setText(f"Temp: {m_temp.group(1)}°C")
                
                # SPAM FILTER: Only print to console if it's NOT coordinates or "ok"
                is_spam = any(x in rdata for x in ["ok", "X:", "Y:", "Z:", "T:0", "B:"])
                if not is_spam: self.console.append(rdata) # Add line to log box
                
            elif rtype == "STATUS":
                self.console.append(f"<b>>>> {rdata}</b>") # Bold status updates
                # If we just connected or had an emergency stop, reset the bar
                if ("CONNECTED" in rdata or "RESET" in rdata or
                        "OFFLINE" in rdata or "SD UPLOAD ERROR" in rdata):
                    self.reset_progress_ui()
            self.console.verticalScrollBar().setValue(self.console.verticalScrollBar().maximum()) # Auto-scroll
    
    def advance_progress(self):
        if self.expected_duration <= 0:  # avoid divsion by 0
            self.move_timer.stop()
            return
    
        elapsed = time.monotonic() - self.start_time
    
        if elapsed >= self.expected_duration:
            # The estimate has elapsed, but the Duet has not yet
            # confirmed completion.
            self.move_timer.stop()
            self.progress_bar.setValue(99)
            self.progress_bar.setFormat("Finishing...")
            return
    
        percentage = int((elapsed / self.expected_duration) * 100)
        percentage = max(0, min(99, percentage))
    
        remaining = max(0.0, self.expected_duration - elapsed)
    
        self.progress_bar.setValue(percentage)
        self.progress_bar.setFormat(f"Moving... {remaining:.1f}s left")
            
    def finish_movement(self):
        """Called after the Duet confirms movement completion."""
        self.move_timer.stop()
        self.expected_duration = 0
        self.stepped_protocol_active = False
        self.protocol_total_cycles = 0
        self.set_motion_active(False)
    
        self.progress_bar.setValue(100)
        self.progress_bar.setFormat("Complete")
    
        # Display Complete for one second, then return to Idle
        self.progress_reset_timer.start(1000)
        
    def reset_progress_ui(self):
        """Force stops the timer and clears the progress bar."""
        self.move_timer.stop()
        self.expected_duration = 0
        self.stepped_protocol_active = False
        self.protocol_total_cycles = 0
        self.set_motion_active(False)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Idle")
        
    def closeEvent(self, event):                         # Runs when user clicks [X]
        # Queue a graceful shutdown. main() waits for the worker and provides
        # a fallback if serial communication prevents a clean exit.
        self.cmd_queue.put(("SHUTDOWN", "KILL"))         # Send M18 and close the serial port
        event.accept()                                   # Close application


def main():
    multiprocessing.freeze_support()

    # Qt and macOS system libraries are not safe with fork. Use one explicit
    # spawn context for the worker and all multiprocessing primitives.
    ctx = multiprocessing.get_context("spawn")

    q1 = ctx.Queue()
    q2 = ctx.Queue()
    ev = ctx.Event()

    worker = ctx.Process(
        target=duet_worker,
        args=(q1, q2, ev),
        daemon=True,
    )
    worker.start()

    app = QApplication(sys.argv)
    gui = StretcherGUI(q1, q2, ev)
    gui.show()

    exit_code = app.exec()

    # closeEvent normally queues SHUTDOWN. Queue it again as a harmless
    # fallback in case Qt exited through app.quit() rather than window close.
    if worker.is_alive():
        q1.put(("SHUTDOWN", "KILL"))

    # Give the worker time to send M18 and close the serial port before using
    # the forced-exit fallback.
    worker.join(timeout=2.0)

    if worker.is_alive():
        ev.set()
        worker.terminate()
        worker.join(timeout=1.0)

    q1.close()
    q2.close()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
