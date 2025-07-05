import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import threading
import robodk.robolink as rdk
import robodk.robomath as rdm
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QCheckBox, QPushButton, QHBoxLayout,
    QVBoxLayout, QWidget, QFrame, QComboBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

# Calibration constants
REAL_TAG_WIDTH = 0.07  # meters
FOCAL_LENGTH = 600     # pixels

# Clamping ranges (adjust as needed to ensure targets are reachable)
X_MIN, X_MAX = 0.15, 0.35
Y_MIN, Y_MAX = -0.1, 0.1
Z_MIN, Z_MAX = 0.05, 0.2

def clamp(value, low, high):
    return max(low, min(value, high))

class AprilTagApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apriltag")
        self.resize(1000, 600)

        self.dark_mode = True
        self.last_move_time = 0
        self.move_delay = 3  # seconds between moves

        # Thread for robot moves (to keep camera live)
        self.move_thread = None

        # --- RoboDK Connection ---
        self.RDK = rdk.Robolink()
        # IMPORTANT: Ensure the string exactly matches your robot name in RoboDK.
        self.robot = self.RDK.Item('Kinova Gen3 lite')
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

        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open camera")
            self.video_label.setText("Camera not plugged in")

        # Initialize AprilTag detector
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.03
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)

        # Database mapping tag IDs to seed types (example)
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

    def move_robot_async(self, x, y, z):
        # If a move is already in progress, skip new command
        if self.move_thread and self.move_thread.is_alive():
            print("Robot move in progress. Skipping new command.")
            return
        self.move_thread = threading.Thread(target=self._move_robot_sync, args=(x, y, z))
        self.move_thread.start()

    def _move_robot_sync(self, x, y, z):
        """
        Moves the robot asynchronously with a fixed (straight) orientation.
        Uses a pure translation (no additional rotation) so the tool stays straight.
        """
        # Build target pose (translation only, no rotation)
        pose_target = rdm.transl(x*1000, y*1000, z*1000)
        try:
            self.robot.MoveJ(pose_target)
        except Exception as e:
            if "Target cannot be reached" in str(e):
                print("Skipping unreachable target.")
                return
            else:
                print("Move error:", e)
                return

        # Close gripper
        self.robot.RunInstruction("GripperClose")

        # Build lift pose (raise 0.1 m)
        pose_lift = rdm.transl(x*1000, y*1000, (z+0.1)*1000)
        try:
            self.robot.MoveJ(pose_lift)
        except Exception as e:
            if "Target cannot be reached" in str(e):
                print("Skipping unreachable lift.")
                return
            else:
                print("Lift error:", e)
                return

        print("Object successfully picked!")

    def update_frame(self):
        if not self.cap.isOpened():
            self.video_label.setText("Camera not plugged in")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.video_label.setText("Camera not plugged in")
            return

        # Optional filtering
        filtered = cv2.bilateralFilter(frame, d=5, sigmaColor=75, sigmaSpace=75)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        height, width = frame.shape[:2]
        center_frame = (width // 2, height // 2)

        # Draw axis lines
        cv2.arrowedLine(frame, (0, center_frame[1]), (width, center_frame[1]), (0,255,255), 3, tipLength=0.05)
        cv2.arrowedLine(frame, (center_frame[0], height), (center_frame[0], 0), (0,255,255), 3, tipLength=0.05)
        cv2.putText(frame, "X", (width - 40, center_frame[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame, "Y", (center_frame[0]+10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.circle(frame, center_frame, 5, (0,255,255), -1)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            now = time.time()
            if now - self.last_move_time > self.move_delay:
                # Use first detected marker
                first_corner = corners[0].reshape((4,2))
                mc = first_corner.mean(axis=0).astype(int)

                # Convert pixel coordinates to robot coordinates (meters)
                alpha = 0.001
                X_0, Y_0, Z_0 = 0.3, 0.0, 0.1
                x_m = X_0 + (320 - mc[0]) * alpha
                y_m = Y_0 + (240 - mc[1]) * alpha
                z_m = Z_0

                # Clamp values to safe workspace
                x_m = clamp(x_m, X_MIN, X_MAX)
                y_m = clamp(y_m, Y_MIN, Y_MAX)
                z_m = clamp(z_m, Z_MIN, Z_MAX)

                # Compute distance (optional)
                d1 = np.linalg.norm(first_corner[0] - first_corner[1])
                d2 = np.linalg.norm(first_corner[1] - first_corner[2])
                measured_width = (d1 + d2) / 2.0
                if measured_width != 0:
                    dist = (REAL_TAG_WIDTH * FOCAL_LENGTH) / measured_width
                    self.distance_label.setText(f"Distance: {dist*100:.1f} cm")
                else:
                    self.distance_label.setText("Distance: N/A")

                # Move robot asynchronously; this won't block the camera feed
                self.move_robot_async(x_m, y_m, z_m)
                self.last_move_time = now

            # Overlay detected markers
            for i, corner in enumerate(corners):
                c = corner.reshape((4,2))
                mc = c.mean(axis=0).astype(int)
                cv2.polylines(frame, [c.astype(int)], isClosed=True, color=(0,255,0), thickness=3)
                cv2.circle(frame, tuple(mc), 5, (0,0,255), -1)
                rx = mc[0] - center_frame[0]
                ry = center_frame[1] - mc[1]
                if int(ids[i][0]) in self.tag_info:
                    seed_type = self.tag_info[int(ids[i][0])]["seed"]
                    text = f"Seed: {seed_type} | ({rx}, {ry})"
                else:
                    text = f"({rx}, {ry})"
                cv2.putText(frame, text, (mc[0]-20, mc[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        else:
            self.distance_label.setText("Distance: N/A")

        # Convert for PyQt display
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
