import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import robodk.robolink as rdk
import robodk.robomath as rdm
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QCheckBox, QPushButton, QHBoxLayout,
    QVBoxLayout, QWidget, QFrame, QComboBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

# Constants from calibration (adjust these with real calibration values)
REAL_TAG_WIDTH = 0.05  # meters
FOCAL_LENGTH = 500     # pixels

# Use these to clamp if desired
X_MIN, X_MAX = 0.1, 0.4
Y_MIN, Y_MAX = -0.2, 0.2
Z_MIN, Z_MAX = 0.05, 0.2

def clamp(value, low, high):
    return max(low, min(value, high))

def make_pose(x_m, y_m, z_m):
    """
    Build a homogeneous transform that places the tool at (x_m, y_m, z_m) in
    robot base coordinates with a fixed “straight” orientation.

    Example: a 180° rotation about X-axis (tool’s Z down).
    Adjust as needed if you want a different orientation.
    """
    # Build translation in mm:
    T = rdm.transl(x_m*1000, y_m*1000, z_m*1000)
    # Multiply by a rotation about X by 180 degrees:
    T *= rdm.rotx(math.radians(180))
    return T

class AprilTagApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apriltag")
        self.resize(1000, 600)

        self.dark_mode = True

        # For robot movement timing
        self.last_move_time = 0
        self.move_delay = 3  # seconds

        # --- RoboDK Connection ---
        self.RDK = rdk.Robolink()
        self.robot = self.RDK.Item('Kinova Gen3 lite')  # Ensure EXACT name
        if not self.robot.Valid():
            print("Error: Robot item 'Kinova Gen3 lite' not found in RoboDK.")
            sys.exit(1)

        # UI Setup
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.hbox = QHBoxLayout(self.central_widget)

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.hbox.addWidget(self.video_label, stretch=3)

        self.control_panel = QFrame()
        self.control_panel.setFrameShape(QFrame.StyledPanel)
        self.vbox = QVBoxLayout(self.control_panel)
        self.vbox.setAlignment(Qt.AlignTop)
        self.vbox.setContentsMargins(20, 20, 20, 20)

        self.title_label = QLabel("Controls")
        self.vbox.addWidget(self.title_label)
        self.vbox.addSpacing(20)

        # Theme switch
        self.theme_layout = QHBoxLayout()
        self.theme_label = QLabel("Dark Mode:")
        self.theme_layout.addWidget(self.theme_label)
        self.theme_switch = QCheckBox()
        self.theme_switch.setObjectName("theme_switch")
        self.theme_switch.setChecked(True)
        self.theme_switch.toggled.connect(self.toggle_theme)
        self.theme_switch.setStyleSheet("""
            QCheckBox#theme_switch { spacing: 5px; }
            QCheckBox#theme_switch::indicator {
                width: 40px; height: 20px; border-radius: 10px;
                background-color: #ccc; border: 1px solid #b3b3b3;
            }
            QCheckBox#theme_switch::indicator:unchecked { background-color: #ccc; margin-left: 1px; }
            QCheckBox#theme_switch::indicator:checked { background-color: #66bb6a; margin-left: 20px; }
        """)
        self.theme_layout.addWidget(self.theme_switch)
        self.theme_layout.addStretch()
        self.vbox.addLayout(self.theme_layout)
        self.vbox.addSpacing(20)

        # Camera selection
        self.camera_layout = QHBoxLayout()
        self.camera_label = QLabel("Select Camera:")
        self.camera_layout.addWidget(self.camera_label)
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(["0", "1"])
        self.camera_layout.addWidget(self.camera_combo)
        self.camera_layout.addStretch()
        self.vbox.addLayout(self.camera_layout)
        self.vbox.addSpacing(10)

        # Distance label
        self.distance_label = QLabel("Distance: N/A")
        self.vbox.addWidget(self.distance_label)
        self.vbox.addSpacing(20)

        # Change camera button
        self.change_camera_button = QPushButton("Change Camera")
        self.change_camera_button.setFixedSize(150, 40)
        self.change_camera_button.clicked.connect(self.change_camera)
        self.vbox.addWidget(self.change_camera_button)
        self.vbox.addSpacing(20)

        # Close button
        self.vbox.addStretch()
        close_layout = QHBoxLayout()
        close_layout.setAlignment(Qt.AlignCenter)
        self.close_button = QPushButton("Close")
        self.close_button.setFixedSize(150, 50)
        self.close_button.clicked.connect(self.close_app)
        close_layout.addWidget(self.close_button)
        self.vbox.addLayout(close_layout)

        self.hbox.addWidget(self.control_panel, stretch=1)

        self.update_theme()

        # Camera init
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open camera")
            self.video_label.setText("Camera not plugged in")

        # AprilTag detector
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.03
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # Tag info (example)
        self.tag_info = {
            1: {"seed": "Porumb"},
            2: {"seed": "Orz"},
            3: {"seed": "Orez"},
            4: {"seed": "Grau"},
            5: {"seed": "Soia"},
            6: {"seed": "Adauga"},
            7: {"seed": "Adauga"},
            8: {"seed": "Adauga"},
            9: {"seed": "Adauga"},
            10: {"seed": "Adauga"}
        }

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def update_theme(self):
        if self.dark_mode:
            self.central_widget.setStyleSheet("background-color: #34495e;")
            self.control_panel.setStyleSheet("background-color: #2c3e50; border: 2px solid white; border-radius: 15px;")
            self.title_label.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")
            self.theme_label.setStyleSheet("color: white; font-size: 16px;")
            self.camera_label.setStyleSheet("color: white; font-size: 16px; margin-right: 10px;")
            self.camera_combo.setStyleSheet("""
                QComboBox {
                    background-color: #2c3e50;
                    color: white;
                    border: 2px solid white;
                    border-radius: 5px;
                    padding: 5px;
                    font-size: 16px;
                }
                QComboBox::drop-down { border: none; }
            """)
            self.change_camera_button.setStyleSheet("""
                QPushButton { background-color: #e67e22; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }
                QPushButton:hover { background-color: #d35400; }
                QPushButton:pressed { background-color: #c0392b; }
            """)
            self.close_button.setStyleSheet("""
                QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }
                QPushButton:hover { background-color: #c0392b; }
                QPushButton:pressed { background-color: #a93226; }
            """)
            self.video_label.setStyleSheet("background: transparent; border: 2px solid white; border-radius: 15px;")
            self.distance_label.setStyleSheet("color: white; font-size: 16px;")
            self.theme_switch.setChecked(True)
        else:
            self.central_widget.setStyleSheet("background-color: #bdc3c7;")
            self.control_panel.setStyleSheet("background-color: #ecf0f1; border: 2px solid black; border-radius: 15px;")
            self.title_label.setStyleSheet("color: black; font-size: 18px; font-weight: bold;")
            self.theme_label.setStyleSheet("color: black; font-size: 16px;")
            self.camera_label.setStyleSheet("color: black; font-size: 16px; margin-right: 10px;")
            self.camera_combo.setStyleSheet("""
                QComboBox {
                    background-color: #ecf0f1;
                    color: black;
                    border: 2px solid black;
                    border-radius: 5px;
                    padding: 5px;
                    font-size: 16px;
                }
                QComboBox::drop-down { border: none; }
            """)
            self.change_camera_button.setStyleSheet("""
                QPushButton { background-color: #e67e22; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }
                QPushButton:hover { background-color: #d35400; }
                QPushButton:pressed { background-color: #c0392b; }
            """)
            self.close_button.setStyleSheet("""
                QPushButton { background-color: #3498db; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; }
                QPushButton:hover { background-color: #2980b9; }
                QPushButton:pressed { background-color: #1c638d; }
            """)
            self.video_label.setStyleSheet("background: transparent; border: 2px solid black; border-radius: 15px;")
            self.distance_label.setStyleSheet("color: black; font-size: 16px;")
            self.theme_switch.setChecked(False)

    def toggle_theme(self, checked):
        self.dark_mode = checked
        self.update_theme()

    def change_camera(self):
        new_index = int(self.camera_combo.currentText())
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(new_index)
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {new_index}")
            self.video_label.setText("Camera not plugged in")
        else:
            self.video_label.setText("")

    def make_pose(self, x_m, y_m, z_m):
        """
        Build a pose that keeps the robot 'straight' (constant orientation).
        This example applies a 180° rotation about the X-axis so the tool faces forward.
        Adjust as needed if you want a different orientation.
        """
        T = rdm.transl(x_m*1000, y_m*1000, z_m*1000)  # translation in mm
        T *= rdm.rotx(math.radians(180))             # rotate 180 deg around X
        return T

    def move_to_tag(self, x, y, z):
        """
        Moves the robot to the given coordinates (in meters) with a fixed orientation.
        """
        pose_target = self.make_pose(x, y, z)
        try:
            self.robot.MoveJ(pose_target)
        except Exception as e:
            print("MoveJ error:", e)
        time.sleep(1)
        self.robot.RunInstruction("GripperClose")
        time.sleep(1)
        # Raise 0.1 m
        pose_lift = self.make_pose(x, y, z+0.1)
        try:
            self.robot.MoveJ(pose_lift)
        except Exception as e:
            print("MoveJ lift error:", e)
        print("Object successfully picked!")

    def update_frame(self):
        if not self.cap.isOpened():
            self.video_label.setText("Camera not plugged in")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.video_label.setText("Camera not plugged in")
            return

        # Optionally filter
        filtered = cv2.bilateralFilter(frame, d=5, sigmaColor=75, sigmaSpace=75)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        height, width = frame.shape[:2]
        center_frame = (width // 2, height // 2)

        # Draw axis lines
        cv2.arrowedLine(frame, (0, center_frame[1]), (width, center_frame[1]), (0,255,255), 3, tipLength=0.05)
        cv2.arrowedLine(frame, (center_frame[0], height), (center_frame[0], 0), (0,255,255), 3, tipLength=0.05)

        cv2.putText(frame, "X", (width - 40, center_frame[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame, "Y", (center_frame[0] + 10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.circle(frame, center_frame, 5, (0,255,255), -1)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            now = time.time()
            if now - self.last_move_time > self.move_delay:
                first_corner = corners[0].reshape((4, 2))
                center = first_corner.mean(axis=0).astype(int)

                # relative pixel coords
                rel_x = center[0] - center_frame[0]
                rel_y = center_frame[1] - center[1]

                # Convert pixel -> meters with offset if you prefer:
                # Example: center of image => (0.3, 0, 0.1)
                # Each pixel => 0.001 m
                X_0, Y_0, Z_0 = 0.3, 0.0, 0.1
                alpha = 0.001
                x_m = X_0 + (320 - center[0]) * alpha
                y_m = Y_0 + (240 - center[1]) * alpha
                z_m = Z_0

                # If you want to clamp:
                x_m = clamp(x_m, 0.1, 0.4)
                y_m = clamp(y_m, -0.2, 0.2)
                z_m = clamp(z_m, 0.05, 0.2)

                # Optionally compute distance
                d1 = np.linalg.norm(first_corner[0] - first_corner[1])
                d2 = np.linalg.norm(first_corner[1] - first_corner[2])
                measured_width = (d1 + d2) / 2.0
                if measured_width != 0:
                    distance = (REAL_TAG_WIDTH * FOCAL_LENGTH) / measured_width
                    distance_cm = distance * 100
                    self.distance_label.setText(f"Distance: {distance_cm:.1f} cm")
                else:
                    self.distance_label.setText("Distance: N/A")

                # Move with fixed orientation
                self.move_to_tag(x_m, y_m, z_m)
                self.last_move_time = now

            # Overlay all detected markers
            for i, corner in enumerate(corners):
                c = corner.reshape((4, 2))
                mc = c.mean(axis=0).astype(int)
                cv2.polylines(frame, [c.astype(int)], isClosed=True, color=(0,255,0), thickness=3)
                cv2.circle(frame, tuple(mc), 5, (0,0,255), -1)

                rel_x = mc[0] - center_frame[0]
                rel_y = center_frame[1] - mc[1]
                if int(ids[i][0]) in self.tag_info:
                    seed_type = self.tag_info[int(ids[i][0])]["seed"]
                    text = f"Seed: {seed_type} | ({rel_x}, {rel_y})"
                else:
                    text = f"({rel_x}, {rel_y})"
                cv2.putText(frame, text, (mc[0]-20, mc[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        else:
            self.distance_label.setText("Distance: N/A")

        # Convert for PyQt
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        bytes_per_line = 3 * width
        qimg = QImage(frame_rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def close_app(self):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AprilTagApp()
    window.show()
    sys.exit(app.exec_())
