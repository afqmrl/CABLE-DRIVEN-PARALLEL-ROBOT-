#!/usr/bin/env python
import os
os.sched_setaffinity(0, {1,2})
import subprocess
import sys
import json
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QDoubleSpinBox, QFormLayout, QTableWidget, QLabel,
    QComboBox, QMessageBox
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import numpy as np
matplotlib.use('Qt5Agg')

# -----------------------------
# Live Plot Canvas (Matplotlib)
# -----------------------------
class LivePlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=8, height=6, dpi=100, motor_ids=None):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super(LivePlotCanvas, self).__init__(self.fig)
        self.setParent(parent)
        self.motor_ids = motor_ids if motor_ids is not None else []
        self.history = {m: [] for m in self.motor_ids}
        self.max_length = 50  # Show only last 10 seconds (0.2s interval)
        self.initialize_plot()
        self.ax.set_autoscale_on(False)  # Disable autoscaling for better performance

    def initialize_plot(self):
        self.ax.set_title("PID Output Data")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Output")
        self.ax.grid(True)
        self.lines = {}
        for m in self.motor_ids:
            line, = self.ax.plot([], [], label=f"Motor {m}")
            self.lines[m] = line
        self.ax.legend()
        self.fig.tight_layout()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-1600, 1600)  # Speed limit is 1500

    def update_plot(self, data):
        try:
            for m in self.motor_ids:
                m_str = str(m)
                if m_str in data.get("pid_outputs", {}):
                    new_data = data["pid_outputs"][m_str]
                    if not isinstance(new_data, list):
                        new_data = [new_data]
                    if new_data:
                        self.history[m].append(new_data[-1])
                        if len(self.history[m]) > self.max_length:
                            self.history[m] = self.history[m][-self.max_length:]
                        xdata = [i * 0.2 for i in range(len(self.history[m]))]  # 0.2s per sample
                        ydata = self.history[m]
                        self.lines[m].set_data(xdata, ydata)
            self.draw()
        except Exception as e:
            print("Error updating plot:", e)

class ForcePlotCanvas(LivePlotCanvas):
    def initialize_plot(self):
        self.ax.set_title("Force Data Plot")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Force Value")
        self.ax.grid(True)
        self.data_keys = ["b1", "b2", "a2", "a1", "b3", "b4", "a4", "a3"]
        self.channels = [
            "Motor_1", "Motor_2", "Motor_3",
            "Motor_4", "Motor_5", "Motor_6",
            "Motor_7", "Motor_8"
        ]
        self.history = {ch: [] for ch in self.channels}
        self.max_length = 50  # Show only last 10 seconds (0.2s interval)
        self.lines = {}
        for ch in self.channels:
            line, = self.ax.plot([], [], label=ch)
            self.lines[ch] = line
        self.ax.legend()
        self.fig.tight_layout()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-10000, 10000)  # Adjusted for your force data range

    def update_plot(self, data):
        try:
            for pretty_name, real_key in zip(self.channels, self.data_keys):
                if real_key in data:
                    new_data = data[real_key]
                    if not isinstance(new_data, list):
                        new_data = [new_data]
                    if new_data:
                        self.history[pretty_name].append(new_data[-1])
                        if len(self.history[pretty_name]) > self.max_length:
                            self.history[pretty_name] = self.history[pretty_name][-self.max_length:]
                        xdata = [i * 0.2 for i in range(len(self.history[pretty_name]))]  # 0.2s per sample
                        ydata = self.history[pretty_name]
                        self.lines[pretty_name].set_data(xdata, ydata)
            self.draw()
        except Exception as e:
            print("Error updating force plot:", e)

class SpeedPlotCanvas(LivePlotCanvas):
    def initialize_plot(self):
        self.ax.set_title("Motor Speed Feedback")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (cm/s)")
        self.ax.grid(True)
        self.lines = {}
        for m in self.motor_ids:
            line, = self.ax.plot([], [], label=f"Motor {m}")
            self.lines[m] = line
        self.ax.legend()
        self.fig.tight_layout()
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-3, 3)  # For ±1500 ticks/s, about ±2.5 cm/s

    def update_plot(self, data):
        try:
            for m in self.motor_ids:
                m_str = str(m)
                if m_str in data.get("motor_speeds", {}):
                    new_data = data["motor_speeds"][m_str]
                    if not isinstance(new_data, list):
                        new_data = [new_data]
                    if new_data:
                        self.history[m].append(new_data[-1])
                        if len(self.history[m]) > self.max_length:
                            self.history[m] = self.history[m][-self.max_length:]
                        xdata = [i * 0.2 for i in range(len(self.history[m]))]
                        DRUM_DIAMETER = 0.022  # meters
                        ENCODER_TICKS_PER_REV = 4096
                        CABLE_CIRCUM = np.pi * DRUM_DIAMETER
                        ydata = [v * CABLE_CIRCUM / ENCODER_TICKS_PER_REV * 100 for v in self.history[m]]  # cm/s
                        self.lines[m].set_data(xdata, ydata)
            self.draw()
        except Exception as e:
            print("Error updating speed plot:", e)

# -----------------------------
# Control Page: Start/Stop, status and motor speeds
# -----------------------------
class ControlPage(QWidget):
    def __init__(self, parent=None):
        super(ControlPage, self).__init__(parent)
        # Use only 4 corners for a square platform
        self.platform_anchors = np.array([
            [ 0.04, -0.04, 0],  # Front right
            [ 0.04,  0.04, 0],  # Back right
            [-0.04,  0.04, 0],  # Back left
            [-0.04, -0.04, 0],  # Front left
        ])
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        # Mode toggle buttons
        mode_layout = QHBoxLayout()
        self.manual_btn = QPushButton("Manual")
        self.joystick_btn = QPushButton("Joystick")
        self.manual_btn.setCheckable(True)
        self.joystick_btn.setCheckable(True)
        self.manual_btn.setChecked(True)
        self.manual_btn.setStyleSheet("QPushButton:checked { background-color: #1976D2; color: white; }")
        self.joystick_btn.setStyleSheet("QPushButton:checked { background-color: #1976D2; color: white; }")
        mode_layout.addStretch()
        mode_layout.addWidget(self.manual_btn)
        mode_layout.addWidget(self.joystick_btn)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)
        layout.addSpacing(20)
        # Centered vertical button layout
        btn_layout = QVBoxLayout()
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.stop_btn.setEnabled(False)
        self.qol_btn = QPushButton("QOL Init")
        self.qol_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        for btn in [self.start_btn, self.stop_btn, self.qol_btn]:
            btn.setFixedWidth(200)
            btn.setMinimumHeight(40)
            btn_layout.addWidget(btn, alignment=QtCore.Qt.AlignHCenter)
            btn_layout.addSpacing(10)
        layout.addLayout(btn_layout)
        layout.addSpacing(20)
        self.status_label = QLabel("Status: Stopped")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.status_label)
        self.speed_label = QLabel("Motor Speeds: N/A")
        self.speed_label.setAlignment(QtCore.Qt.AlignCenter)
        layout.addWidget(self.speed_label)
        # --- New: Desired State and Joystick Status ---
        info_layout = QFormLayout()
        self.joystick_status_label = QLabel("● Disconnected")
        self.joystick_status_label.setStyleSheet("color: red; font-weight: bold;")
        info_layout.addRow("Joystick:", self.joystick_status_label)
        self.desired_pos_label = QLabel("X: 0.00  Y: 0.00  Z: 0.00")
        info_layout.addRow("Platform Desired Pos:", self.desired_pos_label)
        self.desired_rot_label = QLabel("Yaw: 0.00°  Pitch: 0.00°  Roll: 0.00°")
        info_layout.addRow("Platform Desired Rot:", self.desired_rot_label)
        self.platform_xyz_speed_label = QLabel("X: 0.000  Y: 0.000  Z: 0.000 cm/s")
        info_layout.addRow("Platform XYZ Speed:", self.platform_xyz_speed_label)
        self.platform_speed_label = QLabel("0.000 cm/s")
        info_layout.addRow("Platform Speed:", self.platform_speed_label)
        layout.addLayout(info_layout)
        # --- 3D Platform Visualization ---
        self.fig = plt.figure(figsize=(3, 3))
        self.ax3d = self.fig.add_subplot(111, projection='3d')
        self.platform_canvas = FigureCanvas(self.fig)
        layout.addWidget(self.platform_canvas, alignment=QtCore.Qt.AlignHCenter)
        self.draw_platform(np.zeros(3), np.zeros(3))
        # --- End 3D Platform Visualization ---
        self.reset_btn = QPushButton("Reset to (0, 0, 0.3)")
        self.reset_btn.setStyleSheet("background-color: orange; font-weight: bold;")
        self.reset_btn.setVisible(False)
        self.reset_btn.setFixedWidth(200)
        layout.addWidget(self.reset_btn, alignment=QtCore.Qt.AlignHCenter)
        layout.addStretch()

    def draw_platform(self, pos, rot):
        self.ax3d.clear()
        anchors = self.platform_anchors.copy()
        # Apply rotation (yaw, pitch, roll)
        yaw, pitch, roll = np.deg2rad(rot)
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        anchors = (R @ anchors.T).T + pos
        # Draw platform as a square (polygon)
        x = anchors[:, 0]
        y = anchors[:, 1]
        z = anchors[:, 2]
        self.ax3d.plot(np.append(x, x[0]), np.append(y, y[0]), np.append(z, z[0]), 'b-', lw=2)
        self.ax3d.scatter(x, y, z, c='r')
        # Draw axes
        self.ax3d.quiver(pos[0], pos[1], pos[2], 0.03, 0, 0, color='r', linewidth=2)
        self.ax3d.quiver(pos[0], pos[1], pos[2], 0, 0.03, 0, color='g', linewidth=2)
        self.ax3d.quiver(pos[0], pos[1], pos[2], 0, 0, 0.03, color='b', linewidth=2)
        # Set limits and labels
        self.ax3d.set_xlim(-0.1, 0.1)
        self.ax3d.set_ylim(-0.1, 0.1)
        self.ax3d.set_zlim(0, 0.15)
        self.ax3d.set_xlabel('X (m)')
        self.ax3d.set_ylabel('Y (m)')
        self.ax3d.set_zlabel('Z (m)')
        self.ax3d.set_title('Platform Desired Pose')
        self.platform_canvas.draw()

# -----------------------------
# Plot Page: Displays the live PID and force plots
# -----------------------------
class PlotPage(QWidget):
    def __init__(self, motor_ids, parent=None):
        super(PlotPage, self).__init__(parent)
        self.motor_ids = motor_ids
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout()
        self.live_plot = LivePlotCanvas(None, width=8, height=3, dpi=100, motor_ids=self.motor_ids)
        layout.addWidget(self.live_plot)
        self.force_plot = ForcePlotCanvas(None, width=8, height=3, dpi=100)
        layout.addWidget(self.force_plot)
        self.speed_plot = SpeedPlotCanvas(None, width=8, height=3, dpi=100, motor_ids=self.motor_ids)
        layout.addWidget(self.speed_plot)
        self.setLayout(layout)

    def update_plots(self):
        try:
            if os.path.exists("pid_data.json"):
                with open("pid_data.json", "r") as f:
                    pid_data = json.load(f)
                self.live_plot.update_plot(pid_data)
                self.speed_plot.update_plot(pid_data)
            if os.path.exists("force_data.json"):
                with open("force_data.json", "r") as f:
                    force_data = json.load(f)
                self.force_plot.update_plot(force_data)
        except Exception as e:
            print("Error updating plots:", e)

# -----------------------------
# Manual Command Page: Input desired (x, y, z) positions
# -----------------------------
class ManualCommandPage(QWidget):
    def __init__(self, parent=None):
        super(ManualCommandPage, self).__init__(parent)
        self.setup_ui()
        self.saved_dir = "saved_commands"
        os.makedirs(self.saved_dir, exist_ok=True)
        self.load_saved_commands()

    def setup_ui(self):
        main_layout = QHBoxLayout(self)
        left_layout = QVBoxLayout()
        form_layout = QFormLayout()
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-1.0, 1.0)
        self.x_input.setDecimals(3)
        self.x_input.setValue(0.0)
        form_layout.addRow("X (m):", self.x_input)
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-1.0, 1.0)
        self.y_input.setDecimals(3)
        self.y_input.setValue(0.0)
        form_layout.addRow("Y (m):", self.y_input)
        self.z_input = QDoubleSpinBox()
        self.z_input.setRange(0.0, 1.0)
        self.z_input.setDecimals(3)
        self.z_input.setValue(0.3)
        form_layout.addRow("Z (m):", self.z_input)
        left_layout.addLayout(form_layout)
        self.position_table = QTableWidget()
        self.position_table.setColumnCount(3)
        self.position_table.setHorizontalHeaderLabels(["X", "Y", "Z"])
        left_layout.addWidget(self.position_table)
        self.position_table.itemChanged.connect(self.update_3d_plot)
        self.combo_saved = QComboBox()
        self.load_btn = QPushButton("Load Selected")
        self.save_as_btn = QPushButton("Save As New")
        left_layout.addWidget(QLabel("Saved Commands:"))
        left_layout.addWidget(self.combo_saved)
        left_layout.addWidget(self.load_btn)
        left_layout.addWidget(self.save_as_btn)
        self.add_btn = QPushButton("Add Position")
        self.remove_btn = QPushButton("Remove Selected")
        self.save_btn = QPushButton("Save to Robot")
        left_layout.addWidget(self.add_btn)
        left_layout.addWidget(self.remove_btn)
        left_layout.addWidget(self.save_btn)
        right_layout = QVBoxLayout()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvas(self.fig)
        right_layout.addWidget(self.canvas)
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 2)
        # Signals
        self.add_btn.clicked.connect(self.add_position)
        self.remove_btn.clicked.connect(self.remove_selected)
        self.save_btn.clicked.connect(self.save_positions)
        self.load_btn.clicked.connect(self.load_selected_command)
        self.save_as_btn.clicked.connect(self.save_as_new_command)
        self.positions = []
        self.update_3d_plot()

    def add_position(self):
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()
        self.positions.append({"x": x, "y": y, "z": z})
        self.update_table()
        self.update_3d_plot()

    def remove_selected(self):
        selected = self.position_table.currentRow()
        if selected >= 0:
            self.positions.pop(selected)
            self.update_table()
            self.update_3d_plot()

    def save_positions(self):
        if not self.positions:
            QMessageBox.warning(self, "No Positions", "Add at least one position before saving.")
            return
        try:
            with open("desired_positions.json", "w") as f:
                json.dump(self.positions, f)
            QMessageBox.information(self, "Saved", "Commands saved to robot (desired_positions.json).")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save positions:{e}")

    def update_table(self):
        self.position_table.setRowCount(len(self.positions))
        for i, pos in enumerate(self.positions):
            self.position_table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"{pos['x']:.3f}"))
            self.position_table.setItem(i, 1, QtWidgets.QTableWidgetItem(f"{pos['y']:.3f}"))
            self.position_table.setItem(i, 2, QtWidgets.QTableWidgetItem(f"{pos['z']:.3f}"))

    def update_3d_plot(self):
        self.ax.clear()
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Z (m)")
        self.ax.set_title("Waypoints 3D View")
        self.ax.grid(True)
        self.ax.set_box_aspect([1, 1, 0.5])
        xs, ys, zs = [], [], []
        for row in range(self.position_table.rowCount()):
            try:
                x = float(self.position_table.item(row, 0).text())
                y = float(self.position_table.item(row, 1).text())
                z = float(self.position_table.item(row, 2).text())
                xs.append(x)
                ys.append(y)
                zs.append(z)
            except:
                pass
        if xs and ys and zs:
            self.ax.plot(xs, ys, zs, marker='o', linestyle='-')
        self.canvas.draw()

    def load_saved_commands(self):
        self.combo_saved.clear()
        files = [f[:-5] for f in os.listdir(self.saved_dir) if f.endswith(".json")]
        self.combo_saved.addItems(files)

    def load_selected_command(self):
        selected = self.combo_saved.currentText()
        if selected:
            path = os.path.join(self.saved_dir, f"{selected}.json")
            try:
                with open(path, "r") as f:
                    self.positions = json.load(f)
                self.update_table()
                self.update_3d_plot()
                QMessageBox.information(self, "Loaded", f"Loaded {selected}.json")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load {selected}:{e}")

    def save_as_new_command(self):
        # Prompt for a new name and save current positions under saved_dir
        name, ok = QtWidgets.QInputDialog.getText(self, "Save As", "Enter name for this command:")
        if ok and name:
            filename = f"{name}.json"
            path = os.path.join(self.saved_dir, filename)
            try:
                with open(path, "w") as f:
                    json.dump(self.positions, f)
                self.load_saved_commands()
                QMessageBox.information(self, "Saved", f"Saved as {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save as {filename}:{e}")

# -----------------------------
# Main Application Window
# -----------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.input_mode = "manual"  # Ensure input_mode is always defined
        self.setWindowTitle("Motor Control GUI")
        self.resize(1280, 720)
        # track both processes explicitly
        self.main_process = None
        self.force_process = None
        self.qol_process = None
        self.motor_ids = [8, 6, 4, 5, 7, 11, 1, 2]
        self.setup_ui()
        self.setup_timer()
        # Add status bar
        self.statusBar().showMessage("Ready")
        # Add style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QPushButton {
                padding: 5px;
                border-radius: 6px;
                background-color: #2196F3;
                color: white;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:disabled {
                background-color: #BDBDBD;
            }
            QTabWidget::pane {
                border: 1px solid #BDBDBD;
                background: white;
            }
            QTabBar::tab {
                background: #E0E0E0;
                padding: 8px 12px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background: #2196F3;
                color: white;
            }
        """)

    def set_input_mode(self, mode):
        self.input_mode = mode
        self.write_mode_json(mode)
        self.statusBar().showMessage(f"Switched to {mode.capitalize()} mode", 2000)
        # You can adjust which tabs are shown here

    def setup_ui(self):
        # Remove Input Mode menu, use toggle buttons in ControlPage
        self.tabs = QtWidgets.QTabWidget(self)
        self.setCentralWidget(self.tabs)
        self.control_page = ControlPage(self)
        self.plot_page = PlotPage(self.motor_ids, self)
        self.manual_page = ManualCommandPage(self)
        self.tabs.addTab(self.control_page, "Control")
        self.tabs.addTab(self.plot_page, "Live Plot")
        self.tabs.addTab(self.manual_page, "Manual Command")
        # Connect mode toggles
        self.control_page.manual_btn.clicked.connect(lambda: self.set_input_mode("manual"))
        self.control_page.joystick_btn.clicked.connect(lambda: self.set_input_mode("joystick"))
        # Connect buttons
        self.control_page.start_btn.clicked.connect(self.start_main_code)
        self.control_page.stop_btn.clicked.connect(self.stop_main_code)
        self.control_page.reset_btn.clicked.connect(self.reset_position)
        self.control_page.qol_btn.clicked.connect(self.start_qol)

    def setup_timer(self):
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_live_data)
        self.timer.start(500)  # Reduced from 100ms to 200ms for better performance

    def start_force_reader(self):
        try:
            self.force_process = subprocess.Popen([sys.executable, "FORCE_READER_v6.py"])
            self.statusBar().showMessage("Force reader started", 2000)
        except Exception as e:
            self.statusBar().showMessage(f"Error starting force reader: {str(e)}", 3000)

    def start_main_code(self):
        if self.main_process is None:
            try:
                self.main_process = subprocess.Popen([sys.executable, "main_codeV3.py"])
                self.start_force_reader()
                self.control_page.start_btn.setEnabled(False)
                self.control_page.stop_btn.setEnabled(True)
                self.control_page.qol_btn.setVisible(False)
                self.control_page.status_label.setText("Status: Running (Main)")
                self.control_page.reset_btn.setVisible(True)
                self.statusBar().showMessage("Main code started successfully", 3000)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to start main code:\n{e}")
                self.statusBar().showMessage("Failed to start main code", 3000)

    def start_qol(self):
        try:
            # disable the buttons while QOL is running
            self.control_page.start_btn.setEnabled(False)
            self.control_page.stop_btn.setEnabled(True)

            # launch QOL.py and tell it which mode to use
            self.qol_process = subprocess.Popen([
                sys.executable,
                "QOLv3.py",
                "--mode",  self.input_mode  # <-- pass joystick/manual
            ])

            self.control_page.qol_btn.setVisible(False)
            self.control_page.status_label.setText("Status: Running (QOL)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to start QOL script:\n{e}")

    def stop_main_code(self):
        try:
            # 1) Terminate each child if it's still running
            for proc in (self.force_process, self.main_process, self.qol_process):
                if proc is not None:
                    try:
                        if proc.poll() is None:    # still running?
                            proc.terminate()        # send SIGTERM
                            proc.wait(timeout=2)    # wait up to 2 s
                    except Exception:
                        proc.kill()               # force-kill if they ignore terminate()

            # 2) Clear out references
            self.force_process = None
            self.main_process = None
            self.qol_process = None

            # 3) Update UI
            self.control_page.start_btn.setEnabled(True)
            self.control_page.stop_btn.setEnabled(False)
            self.control_page.qol_btn.setVisible(True)
            self.control_page.status_label.setText("Status: Stopped")
            self.control_page.reset_btn.setVisible(False)
            self.statusBar().showMessage("All processes stopped", 3000)
        except Exception as e:
            self.statusBar().showMessage(f"Error stopping processes: {str(e)}", 3000)

    def update_live_data(self):
        try:
            self.plot_page.update_plots()
            motor_speeds = {}
            if os.path.exists("pid_data.json"):
                with open("pid_data.json", "r") as f:
                    data = json.load(f)
                motor_speeds = data.get("motor_speeds", {})
                if motor_speeds:
                    speeds_str = "Motor Speeds: " + ", ".join(
                        [f"{m}: {motor_speeds.get(str(m), 'N/A')}" for m in self.motor_ids]
                    )
                    self.control_page.speed_label.setText(speeds_str)
            # Joystick status
            if os.path.exists("joystick_status.json"):
                with open("joystick_status.json", "r") as f:
                    js = json.load(f)
                if js.get("connected", False):
                    self.control_page.joystick_status_label.setText("● Connected")
                    self.control_page.joystick_status_label.setStyleSheet("color: green; font-weight: bold;")
                else:
                    self.control_page.joystick_status_label.setText("● Disconnected")
                    self.control_page.joystick_status_label.setStyleSheet("color: red; font-weight: bold;")
            # Desired state
            if os.path.exists("desired_state.json"):
                with open("desired_state.json", "r") as f:
                    ds = json.load(f)
                pos = ds.get("position", [0.0, 0.0, 0.0])
                rot = ds.get("rotation", [0.0, 0.0, 0.0])
                self.control_page.desired_pos_label.setText(f"X: {pos[0]:.2f}  Y: {pos[1]:.2f}  Z: {pos[2]:.2f}")
                self.control_page.desired_rot_label.setText(f"Yaw: {rot[0]:.2f}°  Pitch: {rot[1]:.2f}°  Roll: {rot[2]:.2f}°")
                self.control_page.draw_platform(np.array(pos), np.array(rot))
            # Platform speed and xyz speed from platform_state.json
            if os.path.exists("platform_state.json"):
                with open("platform_state.json", "r") as f:
                    state = json.load(f)
                vx, vy, vz = [v * 100 for v in state.get("velocity", [0, 0, 0])]  # convert to cm/s
                speed = state.get("speed", 0) * 100  # convert to cm/s
                self.control_page.platform_xyz_speed_label.setText(
                    f"X: {vx:.3f}  Y: {vy:.3f}  Z: {vz:.3f} cm/s"
                )
                self.control_page.platform_speed_label.setText(f"{speed:.3f} cm/s")
        except Exception as e:
            self.statusBar().showMessage(f"Error updating data: {str(e)}", 3000)

    def reset_position(self):
        try:
            with open("reset_command.json", "w") as f:
                json.dump({"reset": True}, f)
            QMessageBox.information(self, "Reset", "Sent reset command to (0, 0, 0.3)")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to send reset command:\n{e}")

    def closeEvent(self, event):
        try:
            self.stop_main_code()
            event.accept()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error during shutdown: {str(e)}")
            event.accept()

    def write_mode_json(self, mode):
        with open("input_mode.json", "w") as f:
            json.dump({"mode": mode}, f)

    def write_key_json(self, ch):
        path = "keyboard_input.json"
        # initialize if not exists
        if not os.path.exists(path):
            with open(path, "w") as f:
                json.dump([], f)
        # append new key
        with open(path, "r+") as f:
            data = json.load(f)
            data.append(ch)
            f.seek(0)
            json.dump(data, f)
            f.truncate()

    
    def keyPressEvent(self, event):
        ch = event.text()
        if self.input_mode == "manual" and ch:
            self.write_key_json(ch)            # ← record keystroke
        super().keyPressEvent(event)

# -----------------------------
# Main Entry Point
# -----------------------------
if __name__ == "__main__":

     # ensure the JSON files exist
    with open("input_mode.json", "w") as f:
        json.dump({"mode": "manual"}, f)
    with open("keyboard_input.json", "w") as f:
        json.dump([], f)

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
