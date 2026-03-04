# -*- coding: utf-8 -*-
"""
Biaxial Stretcher Control Interface

Created on Wed Mar 4, 2026

Target Hardware: Duet 2 (via USB Serial)

Dependencies: PyQt6, pyserial

@author: urosc + Gemini
"""
# -*- coding: utf-8 -*-
"""
Biaxial Stretcher Control v4.1 | Research Laboratory Edition
-----------------------------------------------------------
Hardware: Duet 2 WiFi/Ethernet (via USB Serial)
Framework: PyQt6 (GUI) + PySerial (Communication)

This script automates a slow-speed biaxial tensile test.
It allows for manual setup (jogging) and an automated protocol
that stops once a target displacement is reached.
"""

import sys
import serial
import serial.tools.list_ports
import time
import multiprocessing
import re
from PyQt6.QtWidgets import (QApplication, QMainWindow, QPushButton, QVBoxLayout, 
                             QHBoxLayout, QWidget, QDoubleSpinBox, QLabel, 
                             QComboBox, QGroupBox, QGridLayout, QSlider)
from PyQt6.QtCore import Qt, QTimer

# =============================================================================
# HARDWARE PROCESS (The "Worker")
# =============================================================================
def duet_worker(cmd_queue, res_queue, stop_event):
    """
    This function runs in a separate CPU process. 
    It manages the serial connection and the movement logic loop.
    """
    ser = None
    experiment_active = False
    exp_params = {"axis": "XY", "speed": 1.0, "target": 0.0}
    
    # Internal tracking of coordinates to determine when to stop
    start_pos = {"X": 0.0, "Y": 0.0}
    current_pos = {"X": 0.0, "Y": 0.0}
    
    while not stop_event.is_set():
        # --- 1. COMMAND PROCESSING ---
        # Check if the GUI has sent any new instructions
        if not cmd_queue.empty():
            msg = cmd_queue.get()
            action, data = msg[0], msg[1]

            if action == "CONNECT":
                try:
                    # Attempt to open the serial port
                    ser = serial.Serial(data, 115200, timeout=0.05)
                    res_queue.put(("STATUS", "CONNECTED"))
                except Exception as e:
                    res_queue.put(("ERROR", str(e)))
            
            elif action == "KILL":
                # M112 is the universal G-code Emergency Stop
                experiment_active = False 
                if ser:
                    ser.write(b"M112\n")
                    ser.flush()
                res_queue.put(("STATUS", "EMERGENCY STOP"))

            elif action == "START_EXP":
                # Initialize the automated stretch
                exp_params = data 
                start_pos = dict(current_pos) # Mark where we started
                experiment_active = True
                res_queue.put(("STATUS", "RUNNING PROTOCOL"))

            elif action == "STOP_EXP":
                # Soft stop: finish current move and wait
                experiment_active = False
                if ser: ser.write(b"M400\n")
                res_queue.put(("STATUS", "PAUSED"))

            elif action == "GCODE" and ser:
                # Direct G-code execution (Homing, Jogging, etc.)
                ser.write(f"{data}\n".encode())
                res = ser.readline().decode().strip()
                if res: 
                    res_queue.put(("DATA", res))
                    # Parse the response to keep 'current_pos' updated
                    m = re.findall(r"([XY]):\s*([-+]?\d*\.\d+|\d+)", res)
                    if m: 
                        for ax, val in m: current_pos[ax] = float(val)

        # --- 2. AUTOMATION LOGIC ---
        # If a protocol is running, calculate if we need to move or stop
        if experiment_active and ser:
            # Calculate absolute distance traveled from the start point
            dist_x = abs(current_pos["X"] - start_pos["X"])
            dist_y = abs(current_pos["Y"] - start_pos["Y"])
            
            # CHECK: Have we reached the user's target displacement?
            if dist_x >= exp_params['target'] or dist_y >= exp_params['target']:
                experiment_active = False
                res_queue.put(("STATUS", "FINISHED: TARGET REACHED"))
                ser.write(b"M400\n") # Ensure buffer is clear
            else:
                # Move in small 0.05mm "chunks"
                # The Duet's motion planner joins these for smooth motion
                speed = exp_params['speed']
                move_axes = " ".join([f"{a}0.05" for a in exp_params['axis']])
                # G91: Relative, G1: Linear Move, G90: Absolute
                ser.write(f"G91\nG1 {move_axes} F{speed}\nG90\n".encode())
                # Sleep briefly to avoid flooding the Duet's serial buffer
                time.sleep(0.1)

        time.sleep(0.01) # Save CPU cycles when idle

# =============================================================================
# GUI PROCESS
# =============================================================================
class StretcherGUI(QMainWindow):
    def __init__(self, cmd_queue, res_queue):
        super().__init__()
        self.cmd_queue = cmd_queue
        self.res_queue = res_queue
        self.setWindowTitle("Biaxial Stretcher v4.1")
        
        # Central widget and main vertical layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # --- 1. CONNECTION UI ---
        conn_group = QGroupBox("Hardware Connection")
        conn_lyt = QHBoxLayout()
        self.port_selector = QComboBox()
        # Automatically list all available COM ports
        self.port_selector.addItems([p.device for p in serial.tools.list_ports.comports()])
        
        btn_conn = QPushButton("Connect USB")
        btn_conn.clicked.connect(lambda: self.cmd_queue.put(("CONNECT", self.port_selector.currentText())))
        
        conn_lyt.addWidget(self.port_selector)
        conn_lyt.addWidget(btn_conn)
        conn_group.setLayout(conn_lyt)
        layout.addWidget(conn_group)

        # --- 2. MANUAL SETUP (JOGGING) ---
        jog_group = QGroupBox("Setup & Manual Positioning")
        jog_layout = QVBoxLayout()
        
        # Grid for X/Y Arrow buttons
        grid = QGridLayout()
        self.add_jog_buttons(grid)
        jog_layout.addLayout(grid)
        
        # Zeroing Button: Critical for starting an experiment at (0,0)
        btn_z = QPushButton("Zero Here (G92)")
        btn_z.setToolTip("Sets current physical position to 0,0 for the software.")
        btn_z.clicked.connect(lambda: self.cmd_queue.put(("GCODE", "G92 X0 Y0")))
        
        jog_layout.addWidget(btn_z)
        jog_group.setLayout(jog_layout)
        layout.addWidget(jog_group)

        # --- 3. PROTOCOL PARAMETERS ---
        proto_group = QGroupBox("Experiment Protocol")
        proto_lyt = QGridLayout()
        
        # Speed: How fast to stretch (mm/min)
        proto_lyt.addWidget(QLabel("Stretch Speed:"), 0, 0)
        self.exp_speed = QDoubleSpinBox()
        self.exp_speed.setRange(0.001, 10.0) # Allows ultra-slow 1 micron/min moves
        self.exp_speed.setDecimals(3)
        self.exp_speed.setValue(1.0)
        self.exp_speed.setSuffix(" mm/min")
        self.exp_speed.valueChanged.connect(self.calculate_time)
        proto_lyt.addWidget(self.exp_speed, 0, 1)

        # Distance: How far to stretch (mm)
        proto_lyt.addWidget(QLabel("Total Displacement:"), 1, 0)
        self.exp_dist = QDoubleSpinBox()
        self.exp_dist.setRange(0.01, 100.0)
        self.exp_dist.setValue(5.0)
        self.exp_dist.setSuffix(" mm")
        self.exp_dist.valueChanged.connect(self.calculate_time)
        proto_lyt.addWidget(self.exp_dist, 1, 1)

        # Time readout: Informational for the researcher
        self.time_info = QLabel("Est. Time: 5.0 min")
        self.time_info.setStyleSheet("color: #1976D2; font-weight: bold;")
        proto_lyt.addWidget(self.time_info, 2, 0, 1, 2)

        # Start/Pause Logic
        btn_start = QPushButton("START PROTOCOL")
        btn_start.setStyleSheet("background-color: #A5D6A7; height: 30px; font-weight: bold;")
        btn_start.clicked.connect(self.start_protocol)
        
        btn_pause = QPushButton("Pause")
        btn_pause.clicked.connect(lambda: self.cmd_queue.put(("STOP_EXP", None)))
        
        proto_lyt.addWidget(btn_start, 3, 0)
        proto_lyt.addWidget(btn_pause, 3, 1)

        proto_group.setLayout(proto_lyt)
        layout.addWidget(proto_group)

        # --- 4. LIVE FEEDBACK ---
        self.status_label = QLabel("Status: Disconnected")
        layout.addWidget(self.status_label)
        
        self.pos_display = QLabel("X: 0.00 | Y: 0.00")
        self.pos_display.setStyleSheet("font-size: 24px; color: #2E7D32; font-family: 'Courier New';")
        self.pos_display.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.pos_display)

        # Kill Switch: Must be accessible at all times
        kill_btn = QPushButton("EMERGENCY STOP")
        kill_btn.setStyleSheet("background-color: #C62828; color: white; font-weight: bold; height: 60px;")
        kill_btn.clicked.connect(lambda: self.cmd_queue.put(("KILL", None)))
        layout.addWidget(kill_btn)

        # UI Timer: Updates the position readout every 250ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_ui)
        self.timer.start(250)

    def calculate_time(self):
        """Calculates estimated duration of the test based on speed and distance."""
        try:
            minutes = self.exp_dist.value() / self.exp_speed.value()
            self.time_info.setText(f"Est. Time: {minutes:.2f} min")
        except ZeroDivisionError:
            self.time_info.setText("Speed cannot be zero")

    def start_protocol(self):
        """Bundles UI parameters and sends them to the hardware process."""
        params = {
            "axis": "XY", 
            "speed": self.exp_speed.value(), 
            "target": self.exp_dist.value()
        }
        self.cmd_queue.put(("START_EXP", params))

    def add_jog_buttons(self, grid):
        """Generates the directional jog buttons for manual setup."""
        dirs = [("Y+", 0, 1, "Y", 1), ("Y-", 2, 1, "Y", -1), 
                ("X-", 1, 0, "X", -1), ("X+", 1, 2, "X", 1)]
        for t, r, c, ax, d in dirs:
            btn = QPushButton(t)
            # Jog move: Relative, F200 (standard setup speed), then back to Absolute
            btn.clicked.connect(lambda ch, a=ax, dist=d: 
                                self.cmd_queue.put(("GCODE", f"G91\nG1 {a}{dist} F200\nG90")))
            grid.addWidget(btn, r, c)

    def refresh_ui(self):
        """Polls the hardware process for current position and status."""
        self.cmd_queue.put(("GCODE", "M114")) # M114 = Get Position
        while not self.res_queue.empty():
            rtype, rdata = self.res_queue.get()
            if rtype == "DATA":
                # Regex parses the Duet's response string for X and Y values
                m = re.findall(r"([XY]):\s*([-+]?\d*\.\d+|\d+)", rdata)
                if len(m) >= 2: 
                    self.pos_display.setText(f"X: {m[0][1]} | Y: {m[1][1]}")
            elif rtype == "STATUS":
                self.status_label.setText(f"Status: {rdata}")

# =============================================================================
# MAIN ENTRY POINT
# =============================================================================
if __name__ == "__main__":
    # Required for Windows multiprocessing compatibility
    multiprocessing.freeze_support()
    
    # Communication channels between GUI and Hardware
    cmd_q = multiprocessing.Queue() # GUI -> Hardware
    res_q = multiprocessing.Queue() # Hardware -> GUI
    
    # Start the hardware-handling process
    p = multiprocessing.Process(target=duet_worker, args=(cmd_q, res_q, multiprocessing.Event()))
    p.daemon = True # Ensures the worker dies when the GUI is closed
    p.start()

    # Launch the PyQt6 Application
    app = QApplication(sys.argv)
    gui = StretcherGUI(cmd_q, res_q)
    gui.show()
    sys.exit(app.exec())