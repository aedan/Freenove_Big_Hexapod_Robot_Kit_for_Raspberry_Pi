# -*- coding: utf-8 -*-
"""
Robot Selector Dialog
Scans network and allows user to select a robot to connect to.
"""

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QListWidget, QListWidgetItem, QProgressBar,
                             QLineEdit, QGroupBox, QMessageBox)
from NetworkScanner import RobotScanner


class ScannerThread(QThread):
    """Background thread for network scanning."""

    progress = pyqtSignal(int, int)  # current, total
    completed = pyqtSignal(list)  # list of robots
    status = pyqtSignal(str)  # status message

    def __init__(self, subnet=None):
        super().__init__()
        self.subnet = subnet
        self.scanner = RobotScanner()

    def run(self):
        """Run network scan in background."""
        self.status.emit("Starting network scan...")

        def progress_callback(current, total):
            self.progress.emit(current, total)

        robots = self.scanner.scan_network(
            subnet=self.subnet,
            progress_callback=progress_callback
        )

        self.completed.emit(robots)


class RobotSelectorDialog(QDialog):
    """
    Dialog for selecting a hexapod robot from the network.
    Features automatic scanning and manual IP entry.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_robot = None
        self.scanner = RobotScanner()
        self.scan_thread = None

        self.setupUI()
        self.load_cached_robots()

    def setupUI(self):
        """Set up the robot selector UI."""
        self.setWindowTitle("Select Hexapod Robot")
        self.setMinimumWidth(600)
        self.setMinimumHeight(500)

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
                padding: 10px;
                color: #DCDCDC;
                background: qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #858585,stop:1 #383838);
                min-height: 35px;
                font-size: 11pt;
            }
            QPushButton:hover {
                color: #000000;
                background-color: #008aff;
            }
            QPushButton:pressed {
                color: #DCDCDC;
                background-color: #444444;
            }
            QPushButton:disabled {
                background: #333333;
                color: #666666;
            }
            QLabel {
                color: #DCDCDC;
                font-size: 10pt;
            }
            QLineEdit {
                border: 1px solid #242424;
                border-radius: 3px;
                padding: 8px;
                background: #383838;
                color: #DCDCDC;
                font-size: 10pt;
            }
            QListWidget {
                border: 1px solid #242424;
                border-radius: 3px;
                background: #383838;
                color: #DCDCDC;
                font-size: 10pt;
            }
            QListWidget::item {
                padding: 10px;
                border-bottom: 1px solid #242424;
            }
            QListWidget::item:selected {
                background: #008aff;
            }
            QListWidget::item:hover {
                background: #555555;
            }
            QProgressBar {
                border: 1px solid #242424;
                border-radius: 3px;
                text-align: center;
                color: #DCDCDC;
                background: #383838;
            }
            QProgressBar::chunk {
                background-color: #00BB9E;
            }
        """)

        # Main layout
        main_layout = QVBoxLayout()

        # Title
        title = QLabel("🤖 Hexapod Robot Network Discovery")
        title.setStyleSheet("font-size: 16pt; font-weight: bold; color: #00BB9E; padding: 10px;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Scan controls
        scan_group = QGroupBox("Network Scan")
        scan_layout = QVBoxLayout()

        scan_btn_layout = QHBoxLayout()
        self.btn_scan = QPushButton("🔍 Scan Network")
        self.btn_scan.clicked.connect(self.start_scan)
        self.btn_refresh = QPushButton("🔄 Refresh")
        self.btn_refresh.clicked.connect(self.refresh_robots)
        scan_btn_layout.addWidget(self.btn_scan)
        scan_btn_layout.addWidget(self.btn_refresh)

        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)

        self.status_label = QLabel("Ready to scan")
        self.status_label.setStyleSheet("color: #00BB9E;")

        scan_layout.addLayout(scan_btn_layout)
        scan_layout.addWidget(self.progress_bar)
        scan_layout.addWidget(self.status_label)
        scan_group.setLayout(scan_layout)
        main_layout.addWidget(scan_group)

        # Robot list
        list_group = QGroupBox("Discovered Robots")
        list_layout = QVBoxLayout()

        self.robot_list = QListWidget()
        self.robot_list.itemDoubleClicked.connect(self.select_robot)

        list_layout.addWidget(self.robot_list)
        list_group.setLayout(list_layout)
        main_layout.addWidget(list_group)

        # Manual IP entry
        manual_group = QGroupBox("Manual Connection")
        manual_layout = QHBoxLayout()

        manual_label = QLabel("IP Address:")
        self.manual_ip_input = QLineEdit()
        self.manual_ip_input.setPlaceholderText("e.g., 192.168.1.100")
        self.btn_manual_connect = QPushButton("Connect")
        self.btn_manual_connect.clicked.connect(self.connect_manual_ip)

        manual_layout.addWidget(manual_label)
        manual_layout.addWidget(self.manual_ip_input)
        manual_layout.addWidget(self.btn_manual_connect)
        manual_group.setLayout(manual_layout)
        main_layout.addWidget(manual_group)

        # Action buttons
        button_layout = QHBoxLayout()
        self.btn_connect = QPushButton("✓ Connect to Selected Robot")
        self.btn_connect.setEnabled(False)
        self.btn_connect.clicked.connect(self.select_robot)
        self.btn_connect.setStyleSheet(
            self.btn_connect.styleSheet() +
            "QPushButton { background-color: #33883; font-weight: bold; }"
        )

        self.btn_cancel = QPushButton("✗ Cancel")
        self.btn_cancel.clicked.connect(self.reject)

        button_layout.addWidget(self.btn_connect)
        button_layout.addWidget(self.btn_cancel)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

        # Enable connect button when robot is selected
        self.robot_list.itemSelectionChanged.connect(self.on_selection_changed)

    def load_cached_robots(self):
        """Load previously discovered robots."""
        cached_robots = self.scanner.load_discovered_robots()
        if cached_robots:
            self.populate_robot_list(cached_robots)
            self.status_label.setText(f"Loaded {len(cached_robots)} cached robot(s)")

    def start_scan(self):
        """Start network scan in background thread."""
        if self.scan_thread and self.scan_thread.isRunning():
            QMessageBox.warning(self, "Scan in Progress", "A scan is already running")
            return

        # Clear current list
        self.robot_list.clear()
        self.btn_scan.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.status_label.setText("Scanning network...")

        # Start scan thread
        self.scan_thread = ScannerThread()
        self.scan_thread.progress.connect(self.on_scan_progress)
        self.scan_thread.completed.connect(self.on_scan_completed)
        self.scan_thread.status.connect(self.on_scan_status)
        self.scan_thread.start()

    def on_scan_progress(self, current, total):
        """Update progress bar during scan."""
        percent = int((current / total) * 100)
        self.progress_bar.setValue(percent)
        self.status_label.setText(f"Scanning... {current}/{total} ({percent}%)")

    def on_scan_completed(self, robots):
        """Handle scan completion."""
        self.btn_scan.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.progress_bar.setVisible(False)

        if robots:
            self.populate_robot_list(robots)
            self.scanner.save_discovered_robots()
            self.status_label.setText(f"Found {len(robots)} robot(s)")
            self.status_label.setStyleSheet("color: #00BB9E;")
        else:
            self.status_label.setText("No robots found")
            self.status_label.setStyleSheet("color: #FF6B6B;")

    def on_scan_status(self, message):
        """Update status label."""
        self.status_label.setText(message)

    def populate_robot_list(self, robots):
        """Populate the robot list widget."""
        self.robot_list.clear()

        for robot in robots:
            # Create list item
            item_text = f"🤖 {robot['name']}\n"
            item_text += f"    IP: {robot['ip']}\n"
            item_text += f"    Type: {robot['type']}\n"
            item_text += f"    Version: {robot['version']}"

            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, robot)  # Store robot data
            self.robot_list.addItem(item)

    def refresh_robots(self):
        """Quick check which cached robots are still reachable."""
        cached_robots = self.scanner.load_discovered_robots()
        if not cached_robots:
            QMessageBox.information(
                self,
                "No Cached Robots",
                "No previously discovered robots. Run a full scan."
            )
            return

        self.status_label.setText("Checking cached robots...")
        self.robot_list.clear()

        reachable = []
        for robot in cached_robots:
            if self.scanner.quick_check_robot(robot['ip']):
                reachable.append(robot)

        if reachable:
            self.populate_robot_list(reachable)
            self.status_label.setText(f"{len(reachable)}/{len(cached_robots)} robot(s) online")
        else:
            self.status_label.setText("No cached robots are reachable")

    def on_selection_changed(self):
        """Enable connect button when robot is selected."""
        has_selection = len(self.robot_list.selectedItems()) > 0
        self.btn_connect.setEnabled(has_selection)

    def select_robot(self):
        """Select the currently highlighted robot."""
        selected_items = self.robot_list.selectedItems()
        if selected_items:
            item = selected_items[0]
            self.selected_robot = item.data(Qt.UserRole)
            self.accept()

    def connect_manual_ip(self):
        """Connect to manually entered IP."""
        ip = self.manual_ip_input.text().strip()
        if not ip:
            QMessageBox.warning(self, "Invalid IP", "Please enter an IP address")
            return

        # Validate IP format (basic check)
        parts = ip.split('.')
        if len(parts) != 4:
            QMessageBox.warning(self, "Invalid IP", "IP address must have 4 octets")
            return

        try:
            for part in parts:
                num = int(part)
                if not (0 <= num <= 255):
                    raise ValueError
        except ValueError:
            QMessageBox.warning(self, "Invalid IP", "Invalid IP address format")
            return

        # Create robot entry for manual IP
        self.selected_robot = {
            "ip": ip,
            "name": f"Robot-{ip.split('.')[-1]}",
            "type": "Manual Entry",
            "version": "Unknown"
        }
        self.accept()

    def get_selected_robot(self):
        """Get the selected robot information."""
        return self.selected_robot


if __name__ == '__main__':
    """Test the robot selector dialog."""
    import sys

    app = QtWidgets.QApplication(sys.argv)
    dialog = RobotSelectorDialog()

    if dialog.exec_() == QDialog.Accepted:
        robot = dialog.get_selected_robot()
        print(f"\nSelected Robot:")
        print(f"  Name: {robot['name']}")
        print(f"  IP: {robot['ip']}")
        print(f"  Type: {robot['type']}")
    else:
        print("Selection cancelled")

    sys.exit(0)
