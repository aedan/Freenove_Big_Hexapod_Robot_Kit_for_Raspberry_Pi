# -*- coding: utf-8 -*-
"""
Autonomous Control Window for Hexapod Robot Client
Provides GUI interface for autonomous features.
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QGroupBox,
                             QPushButton, QLabel, QLineEdit, QTextEdit,
                             QListWidget, QInputDialog, QMessageBox)
from Command import COMMAND as cmd


class AutonomousWindow(QDialog):
    """Dialog window for autonomous robot control."""

    def __init__(self, client, parent=None):
        super(AutonomousWindow, self).__init__(parent)
        self.client = client
        self.setupUI()

    def setupUI(self):
        """Set up the autonomous control UI."""
        self.setWindowTitle("Autonomous Control")
        self.setMinimumWidth(500)
        self.setMinimumHeight(600)

        # Apply dark theme styling
        self.setStyleSheet("""
            QDialog {
                background: #484848;
            }
            QGroupBox {
                color: #DCDCDC;
                border: 2px solid #666666;
                border-radius: 5px;
                margin-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                border-style: none;
                border-radius: 5px;
                padding: 8px;
                color: #DCDCDC;
                background: qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #858585,stop:1 #383838);
                min-height: 30px;
            }
            QPushButton:hover {
                color: #000000;
                background-color: #008aff;
            }
            QPushButton:pressed {
                color: #DCDCDC;
                background-color: #444444;
            }
            QLabel {
                color: #DCDCDC;
            }
            QLineEdit, QTextEdit, QListWidget {
                border: 1px solid #242424;
                border-radius: 3px;
                padding: 5px;
                background: #383838;
                color: #DCDCDC;
                selection-background-color: #008aff;
            }
        """)

        # Main layout
        main_layout = QVBoxLayout()

        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; color: #00BB9E;")
        main_layout.addWidget(self.status_label)

        # Autonomous Mode Control
        mode_group = QGroupBox("Autonomous Modes")
        mode_layout = QVBoxLayout()

        btn_layout_1 = QHBoxLayout()
        self.btn_explore = QPushButton("🔍 Start Exploring")
        self.btn_explore.clicked.connect(self.start_exploring)
        self.btn_patrol = QPushButton("🚨 Start Patrol")
        self.btn_patrol.clicked.connect(self.start_patrol)
        btn_layout_1.addWidget(self.btn_explore)
        btn_layout_1.addWidget(self.btn_patrol)

        self.btn_stop = QPushButton("🛑 Stop Autonomous")
        self.btn_stop.clicked.connect(self.stop_autonomous)
        self.btn_stop.setStyleSheet(self.btn_stop.styleSheet() +
                                    "QPushButton { background-color: #883333; }")

        mode_layout.addLayout(btn_layout_1)
        mode_layout.addWidget(self.btn_stop)
        mode_group.setLayout(mode_layout)
        main_layout.addWidget(mode_group)

        # Face Training Control
        face_group = QGroupBox("Face Recognition Training")
        face_layout = QVBoxLayout()

        btn_layout_2 = QHBoxLayout()
        self.btn_capture_face = QPushButton("📷 Capture Face")
        self.btn_capture_face.clicked.connect(self.capture_face_sample)
        self.btn_train_faces = QPushButton("🎓 Train Recognizer")
        self.btn_train_faces.clicked.connect(self.train_faces)
        btn_layout_2.addWidget(self.btn_capture_face)
        btn_layout_2.addWidget(self.btn_train_faces)

        self.btn_list_faces = QPushButton("📋 List Known Faces")
        self.btn_list_faces.clicked.connect(self.list_known_faces)

        face_layout.addLayout(btn_layout_2)
        face_layout.addWidget(self.btn_list_faces)
        face_group.setLayout(face_layout)
        main_layout.addWidget(face_group)

        # Patrol Waypoint Control
        patrol_group = QGroupBox("Patrol Waypoints")
        patrol_layout = QVBoxLayout()

        waypoint_info = QLabel("Add movement commands as patrol waypoints:")
        waypoint_info.setWordWrap(True)
        patrol_layout.addWidget(waypoint_info)

        btn_layout_3 = QHBoxLayout()
        self.btn_add_forward = QPushButton("➡️ Add Forward")
        self.btn_add_forward.clicked.connect(lambda: self.add_waypoint("forward"))
        self.btn_add_turn_left = QPushButton("↪️ Add Turn Left")
        self.btn_add_turn_left.clicked.connect(lambda: self.add_waypoint("left"))
        self.btn_add_turn_right = QPushButton("↩️ Add Turn Right")
        self.btn_add_turn_right.clicked.connect(lambda: self.add_waypoint("right"))
        btn_layout_3.addWidget(self.btn_add_forward)
        btn_layout_3.addWidget(self.btn_add_turn_left)
        btn_layout_3.addWidget(self.btn_add_turn_right)

        patrol_layout.addLayout(btn_layout_3)
        patrol_group.setLayout(patrol_layout)
        main_layout.addWidget(patrol_group)

        # Auto-Calibration
        calib_group = QGroupBox("Auto-Calibration")
        calib_layout = QVBoxLayout()

        self.btn_auto_calibrate = QPushButton("🎯 Auto-Calibrate & Stand")
        self.btn_auto_calibrate.clicked.connect(self.auto_calibrate)
        calib_layout.addWidget(self.btn_auto_calibrate)

        height_layout = QHBoxLayout()
        height_label = QLabel("Standing Height (inches):")
        self.height_input = QLineEdit("3.0")
        self.height_input.setMaximumWidth(100)
        self.btn_set_height = QPushButton("Set Height")
        self.btn_set_height.clicked.connect(self.set_height)
        height_layout.addWidget(height_label)
        height_layout.addWidget(self.height_input)
        height_layout.addWidget(self.btn_set_height)

        calib_layout.addLayout(height_layout)
        calib_group.setLayout(calib_layout)
        main_layout.addWidget(calib_group)

        # Info/Log area
        log_label = QLabel("Activity Log:")
        main_layout.addWidget(log_label)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        main_layout.addWidget(self.log_text)

        # Close button
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.close)
        main_layout.addWidget(btn_close)

        self.setLayout(main_layout)

    def log_message(self, message):
        """Add message to log area."""
        self.log_text.append(message)
        # Auto-scroll to bottom
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    def start_exploring(self):
        """Start exploration mode."""
        self.client.send_data(cmd.CMD_START_EXPLORING)
        self.status_label.setText("Status: Exploring 🔍")
        self.log_message("Started exploration mode - robot is wandering and learning!")

    def start_patrol(self):
        """Start patrol mode."""
        self.client.send_data(cmd.CMD_START_PATROL)
        self.status_label.setText("Status: Patrolling 🚨")
        self.log_message("Started patrol mode - watching for intruders!")

    def stop_autonomous(self):
        """Stop autonomous operation."""
        self.client.send_data(cmd.CMD_STOP_AUTONOMOUS)
        self.status_label.setText("Status: Idle")
        self.log_message("Stopped autonomous operation")

    def capture_face_sample(self):
        """Capture a face sample for training."""
        name, ok = QInputDialog.getText(
            self,
            "Capture Face Sample",
            "Enter person's name:",
            QtWidgets.QLineEdit.Normal,
            ""
        )

        if ok and name:
            command = f"{cmd.CMD_CAPTURE_FACE_SAMPLE}#{name}"
            self.client.send_data(command)
            self.log_message(f"Captured face sample for '{name}'")
            self.log_message("Tip: Capture 5-10 samples from different angles!")
        else:
            self.log_message("Face capture cancelled")

    def train_faces(self):
        """Train the face recognizer."""
        reply = QMessageBox.question(
            self,
            "Train Face Recognizer",
            "This will train the recognizer with all captured samples.\nContinue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if reply == QMessageBox.Yes:
            self.client.send_data(cmd.CMD_TRAIN_FACES)
            self.log_message("Training face recognizer... (may take a few seconds)")

    def list_known_faces(self):
        """Request list of known faces from server."""
        self.client.send_data(cmd.CMD_LIST_KNOWN_FACES)
        self.log_message("Requested known faces list (check server logs)")

    def add_waypoint(self, direction):
        """Add a patrol waypoint."""
        # Create movement command based on direction
        if direction == "forward":
            waypoint_cmd = "CMD_MOVE#50#0#0#0#tripod"
            self.log_message("Added waypoint: Move Forward")
        elif direction == "left":
            waypoint_cmd = "CMD_MOVE#0#-50#0#0#tripod"
            self.log_message("Added waypoint: Turn Left")
        elif direction == "right":
            waypoint_cmd = "CMD_MOVE#0#50#0#0#tripod"
            self.log_message("Added waypoint: Turn Right")

        command = f"{cmd.CMD_ADD_PATROL_WAYPOINT}#{waypoint_cmd}"
        self.client.send_data(command)

    def auto_calibrate(self):
        """Trigger auto-calibration."""
        reply = QMessageBox.question(
            self,
            "Auto-Calibrate",
            "This will level the robot and lift it to standing height.\n"
            "Make sure the robot is on flat ground.\nContinue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if reply == QMessageBox.Yes:
            self.client.send_data(cmd.CMD_AUTO_CALIBRATE)
            self.log_message("Auto-calibration started...")

    def set_height(self):
        """Set standing height."""
        try:
            height = float(self.height_input.text())
            if height < 1.0 or height > 5.0:
                QMessageBox.warning(
                    self,
                    "Invalid Height",
                    "Height must be between 1.0 and 5.0 inches"
                )
                return

            command = f"{cmd.CMD_SET_HEIGHT}#{height}"
            self.client.send_data(command)
            self.log_message(f"Setting height to {height} inches...")
        except ValueError:
            QMessageBox.warning(
                self,
                "Invalid Input",
                "Please enter a valid number for height"
            )


if __name__ == '__main__':
    """Test the autonomous window."""
    import sys

    class DummyClient:
        def send_data(self, data):
            print(f"Would send: {data}")

    app = QtWidgets.QApplication(sys.argv)
    window = AutonomousWindow(DummyClient())
    window.show()
    sys.exit(app.exec_())
