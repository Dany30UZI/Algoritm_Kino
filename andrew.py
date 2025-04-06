import sys
import os
import csv
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget
)
from PyQt5.QtCore import Qt
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

# Constants
CSV_FILE = 'gen3_lite_poses.csv'

class Gen3LitePoseRecorder(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kinova Gen3 Lite Pose Recorder")
        self.setFixedSize(400, 400)  # Slightly taller to fit more info

        # Central widget and layout
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        self.layout.setAlignment(Qt.AlignCenter)

        # Status label
        self.status_label = QLabel("Move Gen3 Lite and press 'Record' to save pose and gripper")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.layout.addWidget(self.status_label)

        # Last recorded pose label
        self.pose_label = QLabel("Last Recorded: N/A")
        self.pose_label.setAlignment(Qt.AlignCenter)
        self.pose_label.setWordWrap(True)
        self.layout.addWidget(self.pose_label)
        self.layout.addSpacing(20)

        # Record button
        self.record_button = QPushButton("Record Pose")
        self.record_button.setFixedSize(200, 50)
        self.record_button.clicked.connect(self.record_pose)
        self.layout.addWidget(self.record_button, alignment=Qt.AlignCenter)
        self.layout.addSpacing(20)

        # Close button
        self.close_button = QPushButton("Close")
        self.close_button.setFixedSize(200, 50)
        self.close_button.clicked.connect(self.close_app)
        self.layout.addWidget(self.close_button, alignment=Qt.AlignCenter)
        self.layout.addStretch()

        # Styling (simple dark theme)
        self.central_widget.setStyleSheet("background-color: #34495e;")
        self.status_label.setStyleSheet("color: white; font-size: 16px;")
        self.pose_label.setStyleSheet("color: white; font-size: 14px;")
        self.record_button.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }"
            "QPushButton:hover { background-color: #2980b9; }"
        )
        self.close_button.setStyleSheet(
            "QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }"
            "QPushButton:hover { background-color: #c0392b; }"
        )

        # Initialize Kinova Gen3 Lite connection
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            import utilities
            args = utilities.parseConnectionArguments()
            self.device_connection = utilities.DeviceConnection.createTcpConnection(args)
            self.router = self.device_connection.__enter__()
            self.base = BaseClient(self.router)
            print("Connected to Kinova Gen3 Lite successfully")
            servo_mode = Base_pb2.ServoingModeInformation()
            servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servo_mode)
            time.sleep(1)
        except Exception as e:
            print(f"Failed to connect to Kinova Gen3 Lite: {e}")
            self.base = None
            self.status_label.setText("Failed to connect to Gen3 Lite")
            self.record_button.setEnabled(False)

        # Setup CSV file with gripper column
        self.setup_csv()
        self.point_counter = 0

    def setup_csv(self):
        """Initialize CSV file with headers for Gen3 Lite poses and gripper state."""
        try:
            with open(CSV_FILE, 'x', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Point_Number', 'X', 'Y', 'Z', 'Theta_X', 'Theta_Y', 'Theta_Z', 'Gripper'])
        except FileExistsError:
            # Clear existing file and rewrite headers
            with open(CSV_FILE, 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Point_Number', 'X', 'Y', 'Z', 'Theta_X', 'Theta_Y', 'Theta_Z', 'Gripper'])

    def get_pose(self):
        """Retrieve current end effector pose (position and orientation) from Gen3 Lite."""
        if not self.base:
            return None
        try:
            pose = self.base.GetMeasuredCartesianPose()
            return [pose.x, pose.y, pose.z, pose.theta_x, pose.theta_y, pose.theta_z]
        except Exception as e:
            print(f"Error getting Gen3 Lite pose: {e}")
            return None

    def get_gripper_state(self):
        """Retrieve the current gripper position (0 = open, 1 = closed)."""
        if not self.base:
            return None
        try:
            # Create a GripperRequest object
            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION  # Request position mode
            # Get gripper state with the request
            gripper_state = self.base.GetMeasuredGripperMovement(gripper_request)
            # Check the actual structure of the response
            if hasattr(gripper_state, 'finger') and gripper_state.finger:
                return gripper_state.finger[0].value  # Access the first finger's position
            print("No finger data in gripper state response")
            return 0.0  # Default to open if no data
        except Exception as e:
            print(f"Error getting gripper state: {e}")
            return None

    def record_pose(self):
        """Record the current Gen3 Lite pose and gripper state to CSV."""
        if not self.base:
            print("Kinova Gen3 Lite not connected. Cannot record pose.")
            self.status_label.setText("Gen3 Lite not connected")
            return

        pose = self.get_pose()
        gripper_state = self.get_gripper_state()
        if pose and gripper_state is not None:
            self.point_counter += 1
            x, y, z, theta_x, theta_y, theta_z = pose
            with open(CSV_FILE, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.point_counter, x, y, z, theta_x, theta_y, theta_z, gripper_state])
            self.pose_label.setText(
                f"Last Recorded:\nX: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}\n"
                f"θx: {theta_x:.1f}°, θy: {theta_y:.1f}°, θz: {theta_z:.1f}°\n"
                f"Gripper: {gripper_state:.2f} (0 = open, 1 = closed)"
            )
            self.status_label.setText(f"Point {self.point_counter} recorded")
            print(f"Point {self.point_counter} recorded: "
                  f"X={x:.3f}, Y={y:.3f}, Z={z:.3f}, "
                  f"θx={theta_x:.1f}°, θy={theta_y:.1f}°, θz={theta_z:.1f}°, "
                  f"Gripper={gripper_state:.2f}")
        else:
            self.status_label.setText("Failed to record pose or gripper")

    def close_app(self):
        """Close the application and disconnect from the Gen3 Lite."""
        if hasattr(self, 'device_connection'):
            self.device_connection.__exit__(None, None, None)
            print(f"Gen3 Lite poses and gripper states saved to {CSV_FILE}")
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Gen3LitePoseRecorder()
    window.show()
    sys.exit(app.exec_())