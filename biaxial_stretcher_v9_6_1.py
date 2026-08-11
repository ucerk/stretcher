# -*- coding: utf-8 -*-
"""
# =============================================================================
# BIAXIAL STRETCHER CONTROL INTERFACE (v9.6.1)
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
import sys, serial, serial.tools.list_ports, time, multiprocessing, re, math, json
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QHBoxLayout, QWidget, QDoubleSpinBox, QLabel, 
                             QComboBox, QGroupBox, QGridLayout, QTextEdit, 
                             QSlider, QLineEdit, QCheckBox, QMessageBox, QProgressBar)
from PyQt6.QtCore import Qt, QTimer


# RepRapFirmware 3.4.1 absolute minimum vector feed rate.
RRF_MIN_VECTOR_FEEDRATE = 0.60  # mm/min
DIRECT_MIN_SEPARATION_RATE = 2.0 * RRF_MIN_VECTOR_FEEDRATE

# =============================================================================
# OPERATING SYSTEM
# =============================================================================
IS_WINDOWS = sys.platform.startswith("win")
IS_MAC = sys.platform == "darwin"

MONO_FONT = "Menlo" if IS_MAC else "Consolas"

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

class BioSlider(QSlider):
    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Left:
            self.setValue(self.value() - 1)      # -0.01 mm
        elif event.key() == Qt.Key.Key_Right:
            self.setValue(self.value() + 1)      # +0.01 mm
        elif event.key() == Qt.Key.Key_Up:
            self.setValue(self.value() + 10)     # +0.10 mm
        elif event.key() == Qt.Key.Key_Down:
            self.setValue(self.value() - 10)     # -0.10 mm
        else:
            super().keyPressEvent(event)

# =============================================================================
# HARDWARE ENGINE (The Background Worker)
# =============================================================================
def duet_worker(cmd_queue, res_queue, stop_event):
    ser = None
    poll_backoff_until = 0.0

    while not stop_event.is_set():
        try:
            # ============================================================
            # PROCESS COMMAND QUEUE
            # ============================================================
            while not cmd_queue.empty():
                action, data = cmd_queue.get_nowait()

                # ========================================================
                # CONNECT
                # ========================================================
                if action == "CONNECT":
                    try:
                        if ser and ser.is_open:
                            ser.close()

                        ser = None

                        # Open USB serial connection
                        ser = serial.Serial(
                            data,
                            115200,
                            timeout=0.01,
                            write_timeout=1.0
                        )

                        time.sleep(0.5)

                        # Normal connection setup
                        ser.write(
                            b"M552 S0\n"
                            b"M564 H0 S0\n"
                            b"M17\n"
                        )
                        ser.flush()

                        # Clear any previous polling delay
                        poll_backoff_until = 0.0

                        res_queue.put((
                            "STATUS",
                            "CONNECTED"
                        ))

                    except Exception as e:
                        if ser:
                            try:
                                ser.close()
                            except Exception:
                                pass

                        ser = None

                        res_queue.put((
                            "STATUS",
                            f"OFFLINE: {e}"
                        ))

                # ========================================================
                # NORMAL GCODE
                # ========================================================
                elif action == "GCODE":
                    if ser and ser.is_open:
                        try:
                            command = str(data).strip()

                            # IMPORTANT:
                            #
                            # M25/M24 may temporarily make RRF slow to
                            # service the USB stream.
                            #
                            # Set the polling backoff BEFORE sending the
                            # command. The Duet may successfully receive
                            # M25/M24 even if Python subsequently reports
                            # a write timeout.
                            if command in ("M25", "M24"):
                                poll_backoff_until = (
                                    time.monotonic() + 1.0
                                )

                            ser.write(
                                f"{data}\n".encode("utf-8")
                            )
                            ser.flush()

                        except serial.SerialTimeoutException as e:
                            # Do NOT close the connection here.
                            #
                            # A write timeout does not necessarily mean
                            # the Duet disconnected. This is especially
                            # important for M25/M24.
                            res_queue.put((
                                "STATUS",
                                f"COMMAND WRITE TIMEOUT: {e}"
                            ))

                        except (
                            serial.SerialException,
                            PermissionError,
                            OSError
                        ) as e:
                            # Actual serial communication problem
                            res_queue.put((
                                "STATUS",
                                f"SERIAL ERROR: {e}"
                            ))

                # ========================================================
                # SD JOB
                # ========================================================
                elif action == "SD_JOB":
                    if ser and ser.is_open:
                        path, content = data

                        try:
                            # --------------------------------------------
                            # Start SD upload
                            # --------------------------------------------
                            ser.write(
                                f"M28 {path}\n".encode("utf-8")
                            )
                            ser.flush()

                            time.sleep(0.10)

                            # --------------------------------------------
                            # Upload G-code line by line
                            # --------------------------------------------
                            for line in content.splitlines():
                                ser.write(
                                    f"{line}\n".encode("utf-8")
                                )
                                ser.flush()

                                time.sleep(0.005)

                            # --------------------------------------------
                            # Finish upload
                            # --------------------------------------------
                            ser.write(b"M29\n")
                            ser.flush()

                            time.sleep(0.30)

                            # --------------------------------------------
                            # Start SD job
                            # --------------------------------------------
                            ser.write(
                                f"M32 {path}\n".encode("utf-8")
                            )
                            ser.flush()

                        except Exception as e:
                            res_queue.put((
                                "STATUS",
                                f"SD UPLOAD ERROR: {e}"
                            ))

                # ========================================================
                # HARD EMERGENCY STOP
                # ========================================================
                elif action == "ESTOP":
                    if ser and ser.is_open:
                        try:
                            # M112 immediately terminates motion and
                            # shuts down the Duet.
                            ser.write(b"M112\n")
                            ser.flush()

                        except Exception:
                            # USB may become unavailable during M112.
                            pass

                        res_queue.put((
                            "STATUS",
                            "!!! HARD EMERGENCY STOP (M112) !!! "
                            "RESET DUET BEFORE RECONNECTING"
                        ))

                        try:
                            ser.close()
                        except Exception:
                            pass

                        ser = None

                # ========================================================
                # PROGRAM SHUTDOWN
                # ========================================================
                elif action == "SHUTDOWN":
                    if ser and ser.is_open:
                        try:
                            # Disable X/Y motors and remove holding current
                            ser.write(b"M18\n")
                            ser.flush()

                            time.sleep(0.2)

                        except Exception:
                            pass

                        finally:
                            try:
                                ser.close()
                            except Exception:
                                pass

                            ser = None

                    stop_event.set()
                    return

            # ============================================================
            # STATUS POLLING
            # ============================================================
            if ser and ser.is_open:
                try:
                    # ----------------------------------------------------
                    # First read everything the Duet has already sent.
                    # ----------------------------------------------------
                    while ser.in_waiting > 0:
                        line = (
                            ser.readline()
                            .decode(
                                "utf-8",
                                errors="replace"
                            )
                            .strip()
                        )

                        if line:
                            res_queue.put((
                                "DATA",
                                line
                            ))

                    # ----------------------------------------------------
                    # Only send NEW status queries outside the temporary
                    # pause/resume backoff period.
                    # ----------------------------------------------------
                    if time.monotonic() >= poll_backoff_until:
                        try:
                            ser.write(
                                b'M114\n'
                                b'M105\n'
                                b'M409 K"state.status"\n'
                            )
                            ser.flush()

                        except serial.SerialTimeoutException:
                            # IMPORTANT:
                            #
                            # A status-polling timeout does NOT mean that
                            # the USB connection has been lost.
                            #
                            # RRF may simply be busy processing a pause,
                            # resume, or long motion operation.
                            #
                            # Keep the serial port open and retry later.
                            poll_backoff_until = (
                                time.monotonic() + 0.5
                            )

                    time.sleep(0.2)

                # --------------------------------------------------------
                # Actual USB/serial connection failure
                # --------------------------------------------------------
                except (
                    serial.SerialException,
                    PermissionError,
                    OSError
                ) as e:
                    try:
                        ser.close()
                    except Exception:
                        pass

                    ser = None

                    res_queue.put((
                        "STATUS",
                        f"CONNECTION LOST: {e}"
                    ))

        # ================================================================
        # WORKER-LEVEL ERROR
        # ================================================================
        except Exception as e:
            res_queue.put((
                "STATUS",
                f"WORKER ERROR: {e}"
            ))

        time.sleep(0.01)
        
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
        self.motion_is_sd = False
        # SD job state:
        # "running", "pause_requested", "paused", "resume_requested"
        self.sd_pause_state = "running"
        self.pause_started_at = 0.0

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
        self.pos_display.setStyleSheet("font-size: 16px; color: #1B5E20; font-family: {MONO_FONT}; background: #E8F5E9; padding: 5px; border: 1px solid #C8E6C9;")
        self.temp_display = QLabel("Temp: --")           # Create Temperature label
        self.temp_display.setStyleSheet("font-size: 16px; color: #B71C1C; font-family: {MONO_FONT}; background: #FFEBEE; padding: 5px; border: 1px solid #FFCDD2;")
        disp_lyt.addWidget(self.pos_display)             # Add Pos to the row
        disp_lyt.addWidget(self.temp_display)            # Add Temp to the row
        outer_layout.addLayout(disp_lyt)                 # Add the row to the top of window

        # --- SAFETY WARNING ---
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
        self.slider = BioSlider(Qt.Orientation.Horizontal)
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
        self.timed_mode_toggle = QCheckBox("Timed stepped move (SD only)")
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
        sl.addWidget(self.btn_retract)
        sl.addWidget(self.btn_stretch)
        ml.addLayout(sl)
        move_group.setLayout(ml); right_col.addWidget(move_group)
        
        # Pause button is available only for SD-card movements
        self.btn_soft_stop = QPushButton("PAUSE SD MOVEMENT")
        self.btn_soft_stop.setStyleSheet("""
            background-color: #FB8C00;
            color: white;
            font-weight: bold;
            height: 35px;
            border-radius: 5px;
        """)
        
        self.btn_soft_stop.setEnabled(False)
        self.btn_soft_stop.clicked.connect(self.toggle_sd_pause)
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

        self.btn_estop = QPushButton()
        self.btn_estop.clicked.connect(self.emergency_stop)
        outer_layout.addWidget(self.btn_estop)          # Add E-Stop to bottom
        self.set_estop_normal_style()
        
        self.console = QTextEdit()                       # Large black logging box
        self.console.setReadOnly(True); self.console.setMinimumHeight(120)
        self.console.setStyleSheet("background: #1e1e1e; color: #00ff00; font-family: {MONO_FONT}; font-size: 11px;")
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
    
    def emergency_stop(self):
        """Send M112 and tell the user how to recover the Duet."""
        self.cmd_queue.put(("ESTOP", "KILL"))
    
        self.set_estop_triggered_style()
    
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("EMERGENCY STOPPED")
    
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
        
        # Reset pause state when the movement/job is no longer active.
        if not active:
            self.sd_pause_state = "running"
            self.pause_started_at = 0.0
            self.btn_soft_stop.setText("PAUSE SD MOVEMENT")
        
        # Only SD-card movements can be paused/resumed cleanly.
        self.btn_soft_stop.setEnabled(active and self.motion_is_sd)
    
    def configure_segmentation(self, sd_mode, timed_mode):
        """Configure RRF segmentation for the selected movement mode."""
    
        if sd_mode and not timed_mode:
            # Normal continuous movement running from SD.
            # Segment the long G1 so M25 can pause it promptly.
            self.cmd_queue.put(("GCODE", "M669 S20 T0.005"))
    
            self.console.append(
                "<b>SD segmentation enabled: 20 segments/s</b>"
            )
    
        else:
            # Direct USB mode and timed-stepped mode do not need
            # RRF move segmentation.
            self.cmd_queue.put(("GCODE", "M669 S0 T0"))
            
    def on_timed_mode_toggled(self, checked):
        if checked:
            # Remember the user's normal-stretch SD preference
            self.sd_mode_before_timed = self.sd_mode_toggle.isChecked()
    
            # Timed stepped mode must always run from SD
            self.sd_mode_toggle.setChecked(True)
    
        else:
            # Restore the user's previous normal-stretch SD preference
            self.sd_mode_toggle.setChecked(self.sd_mode_before_timed)
    
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
        self.motion_is_sd = True
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
        """Run a direct, SD-macro, or timed-stepped movement."""
    
        # Prevent the previous movement's delayed reset
        # from resetting this new movement.
        self.progress_reset_timer.stop()
    
        # Read the selected operating mode.
        timed_mode = self.timed_mode_toggle.isChecked()
        sd_mode = self.sd_mode_toggle.isChecked()
    
        # ============================================================
        # TIMED STEPPED MODE
        # ============================================================
        if timed_mode:
            # Timed stepped mode already consists of many small moves,
            # so additional RRF segmentation is not required.
            self.cmd_queue.put(("GCODE", "M669 S0 T0"))
    
            self.start_timed_protocol(axis, direction)
            return
    
        # ============================================================
        # NORMAL CONTINUOUS MODE
        # ============================================================
        self.motion_is_sd = sd_mode
        target_dist = self.total_target
    
        # ------------------------------------------------------------
        # Configure RepRapFirmware segmentation
        # ------------------------------------------------------------
        if sd_mode:
            # Normal continuous movement running from SD.
            # The long G1 is internally segmented so that RRF gets
            # regular opportunities to respond to M25 pause requests.
            # S20    = maximum 20 segments per second
            # T0.005 = minimum segment length 0.005 mm
            self.cmd_queue.put(("GCODE", "M669 S20 T0.005"))
    
            self.console.append(
                "<b>SD segmentation enabled: "
                "M669 S20 T0.005</b>"
            )
    
        else:
            # Direct USB movement does not need SD pause segmentation.
            self.cmd_queue.put(("GCODE", "M669 S0 T0"))
    
        # ------------------------------------------------------------
        # Calculate physical movement
        # ------------------------------------------------------------
        # One motor mechanically changes the separation between
        # opposing grippers, therefore each motor moves half of the
        # requested separation.
        motor_move = (target_dist / 2.0) * direction
    
        # GUI speed represents separation speed.
        # Each motor therefore moves at half that speed.
        motor_feedrate = self.speed_box.value() / 2.0
    
        # RepRapFirmware minimum vector feed rate protection.
        if motor_feedrate < RRF_MIN_VECTOR_FEEDRATE:
            QMessageBox.warning(
                self,
                "Rate below firmware minimum",
                "Continuous movement requires at least "
                "1.20 mm/min separation rate. "
                "Use Timed stepped stretch for lower average rates."
            )
            return
    
        # ------------------------------------------------------------
        # Expected movement duration for progress display
        # ------------------------------------------------------------
        self.expected_duration = (
            abs(motor_move) / motor_feedrate
        ) * 60.0
    
        # SD jobs have a small startup delay because the file starts
        # with G4 P500.
        if sd_mode:
            self.expected_duration += 0.5
    
        # ------------------------------------------------------------
        # Lock GUI controls while movement is active
        # ------------------------------------------------------------
        self.set_motion_active(True)
    
        self.start_time = time.monotonic()
    
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("Starting...")
    
        self.move_timer.start(100)
    
        # ------------------------------------------------------------
        # Build movement G-code
        # ------------------------------------------------------------
        if axis == "BOTH":
            # F is vector speed.
            #
            # Multiplying by sqrt(2) keeps each individual X/Y motor
            # moving at motor_feedrate during diagonal XY movement.
            xy_feedrate = motor_feedrate * math.sqrt(2.0)
    
            g_move = (
                "G91\n"
                f"G1 X{motor_move:.6f} "
                f"Y{motor_move:.6f} "
                f"F{xy_feedrate:.3f}\n"
                "G90"
            )
    
        else:
            g_move = (
                "G91\n"
                f"G1 {axis}{motor_move:.6f} "
                f"F{motor_feedrate:.3f}\n"
                "G90"
            )
    
        # ============================================================
        # SD-MACRO MODE
        # ============================================================
        if sd_mode:
            # In SD mode M400 is safe because it runs from the SD-file
            # command stream rather than blocking the USB command stream.
            #
            # This leaves USB available for:
            #   M25 = pause
            #   M24 = resume
            #   M112 = emergency stop
    
            move_sequence = (
                f"{g_move}\n"
                "M400\n"
                'M118 P1 S"__MOVE_DONE__" L0'
            )
    
            # Small startup delay gives the SD job time to enter the
            # processing state cleanly.
            job_content = (
                "G4 P500\n"
                + move_sequence
            )
    
            self.cmd_queue.put((
                "SD_JOB",
                (
                    "0:/gcodes/active.g",
                    job_content
                )
            ))
    
        # ============================================================
        # DIRECT USB MODE
        # ============================================================
        else:
            # IMPORTANT:
            #
            # Do not send M400 here.
            #
            # If M400 is placed on the USB stream, a later M112 could
            # become stuck behind it while the movement is still active.
            self.cmd_queue.put((
                "GCODE",
                g_move
            ))
        
    def toggle_sd_pause(self):
        """Request pause/resume of the active SD job."""
    
        if not self.motion_active or not self.motion_is_sd:
            return
    
        # ============================================================
        # REQUEST PAUSE
        # ============================================================
        if self.sd_pause_state == "running":
    
            self.cmd_queue.put(("GCODE", "M25"))
    
            # Important:
            # Remember that a pause has been REQUESTED.
            # Do not confuse a stale "processing" status with a resume.
            self.sd_pause_state = "pause_requested"
    
            # Freeze estimated progress immediately.
            self.move_timer.stop()
    
            # Start measuring paused time now.
            if self.pause_started_at <= 0:
                self.pause_started_at = time.monotonic()
    
            self.btn_soft_stop.setEnabled(False)
            self.btn_soft_stop.setText("PAUSING...")
            self.progress_bar.setFormat("PAUSING SD JOB...")
    
            self.console.append(
                "<b style='color: #FB8C00;'>"
                ">>> SD JOB PAUSE REQUESTED (M25)"
                "</b>"
            )
    
        # ============================================================
        # REQUEST RESUME
        # ============================================================
        elif self.sd_pause_state == "paused":
    
            self.cmd_queue.put(("GCODE", "M24"))
    
            self.sd_pause_state = "resume_requested"
    
            self.btn_soft_stop.setEnabled(False)
            self.btn_soft_stop.setText("RESUMING...")
            self.progress_bar.setFormat("RESUMING SD JOB...")
    
            self.console.append(
                "<b style='color: #388E3C;'>"
                ">>> SD JOB RESUME REQUESTED (M24)"
                "</b>"
            )
    
        # If pause/resume is already in transition,
        # ignore additional clicks.
    
    def sync_duet_job_state(self, status):
        """Synchronize GUI state with the actual Duet SD-job state."""
    
        status = status.lower()
    
        if not self.motion_active or not self.motion_is_sd:
            return
    
        # ============================================================
        # DUET IS PAUSING
        # ============================================================
        if status == "pausing":
    
            # This may have been initiated by either:
            #   - GUI M25
            #   - PanelDue
            if self.sd_pause_state == "running":
                self.sd_pause_state = "pause_requested"
    
                if self.pause_started_at <= 0:
                    self.pause_started_at = time.monotonic()
    
            # Freeze normal estimated progress.
            self.move_timer.stop()
    
            self.btn_soft_stop.setEnabled(False)
            self.btn_soft_stop.setText("PAUSING...")
            self.progress_bar.setFormat("PAUSING SD JOB...")
    
        # ============================================================
        # DUET IS PAUSED
        # ============================================================
        elif status == "paused":
    
            # PanelDue may have initiated the pause, so make sure
            # we have a pause timestamp.
            if self.pause_started_at <= 0:
                self.pause_started_at = time.monotonic()
    
            self.sd_pause_state = "paused"
    
            # Stop normal movement progress.
            self.move_timer.stop()
    
            self.btn_soft_stop.setEnabled(True)
            self.btn_soft_stop.setText("RESUME SD MOVEMENT")
            self.progress_bar.setFormat("SD JOB PAUSED")
    
        # ============================================================
        # DUET IS RESUMING
        # ============================================================
        elif status == "resuming":
    
            self.sd_pause_state = "resume_requested"
    
            self.btn_soft_stop.setEnabled(False)
            self.btn_soft_stop.setText("RESUMING...")
            self.progress_bar.setFormat("RESUMING SD JOB...")
    
        # ============================================================
        # DUET IS PROCESSING THE SD JOB
        # ============================================================
        elif status == "processing":
    
            # --------------------------------------------------------
            # CRITICAL:
            #
            # Immediately after sending M25, we may still receive an
            # old/stale "processing" response before RRF changes to
            # "pausing" or "paused".
            #
            # DO NOT interpret that as a resume.
            # --------------------------------------------------------
            if self.sd_pause_state == "pause_requested":
                return
    
            # --------------------------------------------------------
            # Actual resume
            # --------------------------------------------------------
            if self.sd_pause_state in (
                "paused",
                "resume_requested"
            ):
    
                # Remove paused time from the progress estimate.
                if self.pause_started_at > 0:
                    paused_duration = (
                        time.monotonic()
                        - self.pause_started_at
                    )
    
                    self.start_time += paused_duration
    
                self.pause_started_at = 0.0
                self.sd_pause_state = "running"
    
                # Normal SD movement uses estimated time progress.
                # Timed stepped mode gets progress from
                # __STEP_PROGRESS__ messages instead.
                if (
                    self.expected_duration > 0
                    and not self.stepped_protocol_active
                ):
                    self.move_timer.start(100)
    
            self.btn_soft_stop.setEnabled(True)
            self.btn_soft_stop.setText("PAUSE SD MOVEMENT")    
        
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
        while not self.res_queue.empty():                # Process all hardware messages in queue
            rtype, rdata = self.res_queue.get_nowait()
            if rtype == "DATA":
                # Synchronize SD pause/resume state with the Duet Object Model.
                if '"state.status"' in rdata:
                    try:
                        reply = json.loads(rdata)
             
                        if reply.get("key") == "state.status":
                            duet_status = reply.get("result")
             
                            if isinstance(duet_status, str):
                                self.sync_duet_job_state(duet_status)
             
                            # Hide repetitive status polling from console.
                            continue
             
                    except json.JSONDecodeError:
                        pass
            
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
                self.console.append(f"<b>>>> {rdata}</b>")
            
                if "CONNECTED" in rdata:
                    self.set_estop_normal_style()
                    self.btn_estop.setEnabled(True)
            
                if ("CONNECTED" in rdata
                    or "RESET" in rdata
                    or "OFFLINE" in rdata
                    or "SD UPLOAD ERROR" in rdata):
                    self.reset_progress_ui()
            self.console.verticalScrollBar().setValue(self.console.verticalScrollBar().maximum()) # Auto-scroll
    
    def set_estop_normal_style(self):
        """Restore the normal emergency-stop button appearance."""
    
        self.btn_estop.setText("HARD EMERGENCY STOP (M112)")
        self.btn_estop.setEnabled(True)
    
        self.btn_estop.setStyleSheet("""QPushButton {background-color: #B71C1C;
                                                     color: white;
                                                     font-weight: bold;
                                                     height: 45px;
                                                     border-radius: 5px;}
    
                                        QPushButton:hover {background-color: #E53935;}
                                
                                        QPushButton:pressed {background-color: #FFF3E0;}""")
    
    
    def set_estop_triggered_style(self):
        """Show that M112 was triggered and PanelDue STOP is required."""
    
        self.btn_estop.setText("EMERGENCY STOPPED - PRESS STOP ON PANEL DUE BEFORE RECONNECTING")
        self.btn_estop.setEnabled(False)
    
        self.btn_estop.setStyleSheet("""QPushButton {background-color: #FFF3E0;
                                                     color: #D84315;
                                                     font-weight: bold;
                                                     height: 45px;
                                                     border-radius: 5px;}
                                        
                                            QPushButton:disabled {background-color: #FFF3E0;
                                                                  color: #D84315;}""")
    
    def advance_progress(self):
        if self.expected_duration <= 0:  # avoid divsion by 0
            self.move_timer.stop()
            return
    
        elapsed = time.monotonic() - self.start_time
    
        if elapsed >= self.expected_duration:
            self.move_timer.stop()
        
            if not self.motion_is_sd:
                # Direct USB move has no blocking M400 completion marker.
                # Use the calculated movement duration for GUI completion.
                self.finish_movement()
            else:
                # SD job will give us __MOVE_DONE__ after its M400.
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
        """Turn off the motors before closing the application."""
        self.cmd_queue.put(("SHUTDOWN", "KILL"))         # Ask the serial worker to send M18 first.
        self.stop_event.wait(timeout=1.0)                # Wait until the worker has sent M18 and closed the serial port.
    
        event.accept()

if __name__ == "__main__":
    multiprocessing.freeze_support()                     # Support for .exe bundling
    q1, q2, ev = multiprocessing.Queue(), multiprocessing.Queue(), multiprocessing.Event()
    multiprocessing.Process(target=duet_worker, args=(q1, q2, ev), daemon=True).start() # Start hardware process
    app = QApplication(sys.argv); gui = StretcherGUI(q1, q2, ev); gui.show(); sys.exit(app.exec()) # Start GUI