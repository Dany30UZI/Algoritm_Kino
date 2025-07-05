import sys
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import threading
import grpc
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QCheckBox, QPushButton, QHBoxLayout,
    QVBoxLayout, QWidget, QFrame, QComboBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

# Calibration constants (adjust as needed)
REAL_TAG_WIDTH = 0.05   # meters (example value)
FOCAL_LENGTH = 500      # pixels (example value)

# Clamping ranges (to keep poses within a reachable workspace)
X_MIN, X_MAX = 0.1, 0.4
Y_MIN, Y_MAX = -0.2, 0.2
Z_MIN, Z_MAX = 0.05, 0.2

# Desired forward distance from the tag (15 cm)
DESIRED_DISTANCE = 0.15  # meters

def clamp(value, low, high):
    return max(low, min(value, high))

def make_pose(x_m, y_m, z_m, angle_deg=0):
    """
    Build a 4x4 homogeneous transform that places the tool at (x_m, y_m, z_m)
    in robot base coordinates. It applies a 180° rotation about X (to fix a baseline
    orientation so the tool faces forward) and then a rotation about Z by angle_deg to
    account for the tag's in-plane rotation. Translations are in meters (converted to mm).
    """
    T = np.eye(4)
    T[0, 3] = x_m * 1000
    T[1, 3] = y_m * 1000
    T[2, 3] = z_m * 1000
    # 180° rotation about X
    Rx = np.array([[1, 0, 0, 0],
                   [0, math.cos(math.radians(180)), -math.sin(math.radians(180)), 0],
                   [0, math.sin(math.radians(180)), math.cos(math.radians(180)), 0],
                   [0, 0, 0, 1]])
    # Rotation about Z by angle_deg:
    Rz = np.array([[math.cos(math.radians(angle_deg)), -math.sin(math.radians(angle_deg)), 0, 0],
                   [math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg)), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    T = T.dot(Rx).dot(Rz)
    return T

def matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix R into a quaternion (qw, qx, qy, qz).
    """
    trace = np.trace(R)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2,1] - R[1,2]) / s
        qy = (R[0,2] - R[2,0]) / s
        qz = (R[1,0] - R[0,1]) / s
    else:
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            qw = (R[2,1] - R[1,2]) / s
            qx = 0.25 * s
            qy = (R[0,1] + R[1,0]) / s
            qz = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            qw = (R[0,2] - R[2,0]) / s
            qx = (R[0,1] + R[1,0]) / s
            qy = 0.25 * s
            qz = (R[1,2] + R[2,1]) / s
        else:
            s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            qw = (R[1,0] - R[0,1]) / s
            qx = (R[0,2] + R[2,0]) / s
            qy = (R[1,2] + R[2,1]) / s
            qz = 0.25 * s
    return qw, qx, qy, qz

class AprilTagApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apriltag")
        self.resize(1000, 600)

        self.dark_mode = True
        self.last_move_time = 0
        self.move_delay = 3  # seconds delay between moves
        self.move_thread = None  # for asynchronous robot movement
        self.last_angle = None  # store last computed angle

        # --- Kinova KORTEX Connection ---
        # Replace with your robot's IP address and correct port (typically 50051)
        robot_ip = "192.168.1.10"  # Update as needed
        self.channel = grpc.insecure_channel(f"{robot_ip}:50051")
        self.base_client = BaseClient(self.channel)

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

        # Theme switch layout
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

        # Camera selection layout
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
        self.vbox.addSpacing(10)
        
        # Angle label (persistent display)
        self.angle_label = QLabel("Angle: N/A")
        self.vbox.addWidget(self.angle_label)
        self.vbox.addSpacing(20)

        # Change camera button
        self.change_camera_button = QPushButton("Change Camera")
        self.change_camera_button.setFixedSize(150, 40)
        self.change_camera_button.clicked.connect(self.change_camera)
        self.vbox.addWidget(self.change_camera_button)
        self.vbox.addSpacing(20)

        # Centered Close button
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

        # Camera initialization
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open camera")
            self.video_label.setText("Camera not plugged in")

        # AprilTag detector setup
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
        self.timer.start(30)  # ~33 fps

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
            self.angle_label.setStyleSheet("color: white; font-size: 16px;")
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
            self.angle_label.setStyleSheet("color: black; font-size: 16px;")
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

    def move_robot_async(self, x, y, z, angle_deg):
        if self.move_thread and self.move_thread.is_alive():
            print("Robot move in progress. Skipping new command.")
            return
        self.move_thread = threading.Thread(target=self._move_robot_sync, args=(x, y, z, angle_deg))
        self.move_thread.start()

    def _move_robot_sync(self, x, y, z, angle_deg):
        # Build the target pose matrix using our helper function.
        target_pose = make_pose(x, y, z, angle_deg)
        pos = target_pose[0:3, 3]  # position (in mm)
        R = target_pose[0:3, 0:3]  # rotation matrix
        qw, qx, qy, qz = matrix_to_quaternion(R)
        
        # Build the Cartesian trajectory request for the Kinova KORTEX API.
        request = Base_pb2.PlayCartesianTrajectoryRequest()
        request.constraint.pose.position.x = pos[0]
        request.constraint.pose.position.y = pos[1]
        request.constraint.pose.position.z = pos[2]
        request.constraint.pose.orientation.x = qx
        request.constraint.pose.orientation.y = qy
        request.constraint.pose.orientation.z = qz
        request.constraint.pose.orientation.w = qw
        try:
            self.base_client.PlayCartesianTrajectory(request)
        except Exception as e:
            print("Move error (Kortex):", e)
            return

        print("Robot moved to target pose.")

    def update_frame(self):
        if not self.cap.isOpened():
            self.video_label.setText("Camera not plugged in")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.video_label.setText("Camera not plugged in")
            return

        filtered = cv2.bilateralFilter(frame, d=5, sigmaColor=75, sigmaSpace=75)
        gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)

        height, width = frame.shape[:2]
        center_frame = (width // 2, height // 2)

        cv2.arrowedLine(frame, (0, center_frame[1]), (width, center_frame[1]), (0,255,255), 3, tipLength=0.05)
        cv2.arrowedLine(frame, (center_frame[0], height), (center_frame[0], 0), (0,255,255), 3, tipLength=0.05)
        cv2.putText(frame, "X", (width-40, center_frame[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.putText(frame, "Y", (center_frame[0]+10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
        cv2.circle(frame, center_frame, 5, (0,255,255), -1)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            now = time.time()
            if now - self.last_move_time > self.move_delay:
                first_corner = corners[0].reshape((4, 2))
                center = first_corner.mean(axis=0).astype(int)
                # Compute the in-plane angle from the first two corners:
                angle_rad = math.atan2(first_corner[1][1] - first_corner[0][1],
                                       first_corner[1][0] - first_corner[0][0])
                angle_deg = math.degrees(angle_rad)
                self.last_angle = angle_deg
                self.angle_label.setText(f"Angle: {angle_deg:.1f} deg")
                
                # Highlight the reference line in purple:
                pt1 = tuple(first_corner[0].astype(int))
                pt2 = tuple(first_corner[1].astype(int))
                cv2.line(frame, pt1, pt2, (255, 0, 255), 2)
                ref_pt = ((first_corner[0] + first_corner[1]) / 2).astype(int)
                cv2.circle(frame, tuple(ref_pt), 5, (255, 0, 255), -1)
                cv2.putText(frame, "Ref", (ref_pt[0] + 5, ref_pt[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                
                # Compute distance using the pinhole camera model:
                d1 = np.linalg.norm(first_corner[0] - first_corner[1])
                d2 = np.linalg.norm(first_corner[1] - first_corner[2])
                measured_width = (d1 + d2) / 2.0
                if measured_width != 0:
                    computed_distance = (REAL_TAG_WIDTH * FOCAL_LENGTH) / measured_width  # in meters
                    self.distance_label.setText(f"Distance: {computed_distance*100:.1f} cm")
                else:
                    computed_distance = None
                    self.distance_label.setText("Distance: N/A")
                
                # Set target x coordinate: if the tag is farther than desired,
                # command the robot so that its x-position becomes DESIRED_DISTANCE (15 cm)
                if computed_distance is not None and computed_distance > DESIRED_DISTANCE:
                    x_target = DESIRED_DISTANCE
                else:
                    x_target = 0.3  # fallback if tag is too close

                # Lateral coordinate: compute from image center offset.
                Y_0 = 0.0
                alpha = 0.001
                y_target = Y_0 + (240 - center[1]) * alpha
                # Fixed z coordinate:
                z_target = 0.1

                self.move_robot_async(x_target, y_target, z_target, angle_deg)
                self.last_move_time = now

            for i, corner in enumerate(corners):
                c = corner.reshape((4, 2))
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
                cv2.putText(frame, text, (mc[0]-20, mc[1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        else:
            self.distance_label.setText("Distance: N/A")
            self.angle_label.setText("Angle: N/A")

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
