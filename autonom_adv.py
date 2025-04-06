import sys
import cv2
import cv2.aruco as aruco
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QLabel, QCheckBox, QPushButton, QHBoxLayout,
    QVBoxLayout, QWidget, QFrame, QComboBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
import threading
import os
import time
import math

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

# Constants
REAL_TAG_WIDTH = 0.05
FOCAL_LENGTH = 550
TARGET_DISTANCE_CM = 20
SPEED_FACTOR = 0.3
P_GAIN = 0.1           # Strong correction
DEADBAND = 0.02        # Tight alignment (~1.1°)
MAX_ANGULAR_VEL = 0.5  # Limit angular velocity (rad/s)
TIMEOUT_DURATION = 30
SEARCH_TIMEOUT = 5
SEARCH_SPEED = 0.02
CENTER_TOLERANCE = 20
CAMERA_OFFSET_X = 0.0
CAMERA_OFFSET_Y = 0.0
JOINT_SPEED = 10.0
JOINT_TOLERANCE = 2.0

CAMERA_MATRIX = np.array([[FOCAL_LENGTH, 0, 320], [0, FOCAL_LENGTH, 240], [0, 0, 1]], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)

class AprilTagApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AprilTag Alignment")
        self.resize(1000, 600)
        self.dark_mode = True

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

        self.theme_layout = QHBoxLayout()
        self.theme_label = QLabel("Dark Mode:")
        self.theme_layout.addWidget(self.theme_label)
        self.theme_switch = QCheckBox()
        self.theme_switch.setChecked(True)
        self.theme_switch.toggled.connect(self.toggle_theme)
        self.theme_layout.addWidget(self.theme_switch)
        self.theme_layout.addStretch()
        self.vbox.addLayout(self.theme_layout)
        self.vbox.addSpacing(20)

        self.camera_layout = QHBoxLayout()
        self.camera_label = QLabel("Select Camera:")
        self.camera_layout.addWidget(self.camera_label)
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(["0", "1"])
        self.camera_layout.addWidget(self.camera_combo)
        self.camera_layout.addStretch()
        self.vbox.addLayout(self.camera_layout)
        self.vbox.addSpacing(10)

        self.distance_label = QLabel("Distance: N/A")
        self.vbox.addWidget(self.distance_label)
        self.vbox.addSpacing(20)

        self.change_camera_button = QPushButton("Change Camera")
        self.change_camera_button.clicked.connect(self.change_camera)
        self.vbox.addWidget(self.change_camera_button)
        self.vbox.addSpacing(20)

        self.autonomy_button = QPushButton("Toggle Autonomy")
        self.autonomy_button.clicked.connect(self.toggle_autonomy)
        self.vbox.addWidget(self.autonomy_button)
        self.vbox.addSpacing(20)

        self.reset_robot_button = QPushButton("Reset Robot")
        self.reset_robot_button.clicked.connect(self.reset_robot)
        self.vbox.addWidget(self.reset_robot_button)
        self.vbox.addSpacing(20)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.clicked.connect(self.stop_robot)
        self.vbox.addWidget(self.stop_robot_button)
        self.vbox.addSpacing(20)

        self.close_button = QPushButton("Close")
        self.close_button.clicked.connect(self.close_app)
        self.vbox.addStretch()
        self.vbox.addWidget(self.close_button, alignment=Qt.AlignCenter)

        self.hbox.addWidget(self.control_panel, stretch=1)
        self.update_theme()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open camera")
            self.video_label.setText("Camera not plugged in")

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters()
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.03
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.03
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.tag_info = {
            1: {"seed": "Porumb"}, 2: {"seed": "Orz"}, 3: {"seed": "Orez"},
            4: {"seed": "Grau"}, 5: {"seed": "Soia"}, 6: {"seed": "Adauga"},
            7: {"seed": "Adauga"}, 8: {"seed": "Adauga"}, 9: {"seed": "Adauga"},
            10: {"seed": "Adauga"}
        }

        self.last_tag_position = None
        self.last_tag_pose = None
        self.initial_joint_angles = None
        self.is_autonomous = False
        self.last_tag_time = time.time()
        self.search_angle = 0.0

        try:
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            import utilities
            args = utilities.parseConnectionArguments()
            self.device_connection = utilities.DeviceConnection.createTcpConnection(args)
            self.router = self.device_connection.__enter__()
            self.base = BaseClient(self.router)
            print("Connected to Kinova arm")
            servo_mode = Base_pb2.ServoingModeInformation()
            servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servo_mode)
            time.sleep(1)
            self.initial_joint_angles = self.get_current_joint_angles()
            if not self.initial_joint_angles:
                raise Exception("Failed to retrieve initial joint angles")
            print(f"Initial joint angles: {self.initial_joint_angles}")
        except Exception as e:
            print(f"Failed to connect to Kinova arm: {e}")
            self.base = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

        threading.Thread(target=self.autonomous_loop, daemon=True).start()

    def update_theme(self):
        if self.dark_mode:
            self.central_widget.setStyleSheet("background-color: #34495e;")
            self.control_panel.setStyleSheet("background-color: #2c3e50; border: 2px solid white;")
            self.title_label.setStyleSheet("color: white; font-size: 18px;")
            self.theme_label.setStyleSheet("color: white;")
            self.camera_label.setStyleSheet("color: white;")
            self.camera_combo.setStyleSheet(
                "QComboBox { background-color: #2c3e50; color: white; border: 1px solid white; padding: 5px; }"
                "QComboBox::drop-down { border: none; }"
            )
            self.distance_label.setStyleSheet("color: white;")
            self.change_camera_button.setStyleSheet("background-color: #e67e22; color: white; border: none;")
            self.autonomy_button.setStyleSheet("background-color: #3498db; color: white; border: none;")
            self.reset_robot_button.setStyleSheet("background-color: #f39c12; color: white; border: none;")
            self.stop_robot_button.setStyleSheet("background-color: #e74c3c; color: white; border: none;")
            self.close_button.setStyleSheet("background-color: #e74c3c; color: white; border: none;")
            self.video_label.setStyleSheet("border: 2px solid white;")
        else:
            self.central_widget.setStyleSheet("background-color: #bdc3c7;")
            self.control_panel.setStyleSheet("background-color: #ecf0f1; border: 2px solid black;")
            self.title_label.setStyleSheet("color: black; font-size: 18px;")
            self.theme_label.setStyleSheet("color: black;")
            self.camera_label.setStyleSheet("color: black;")
            self.camera_combo.setStyleSheet(
                "QComboBox { background-color: #ecf0f1; color: black; border: 1px solid black; padding: 5px; }"
                "QComboBox::drop-down { border: none; }"
            )
            self.distance_label.setStyleSheet("color: black;")
            self.change_camera_button.setStyleSheet("background-color: #e67e22; color: white; border: none;")
            self.autonomy_button.setStyleSheet("background-color: #3498db; color: white; border: none;")
            self.reset_robot_button.setStyleSheet("background-color: #f39c12; color: white; border: none;")
            self.stop_robot_button.setStyleSheet("background-color: #e74c3c; color: white; border: none;")
            self.close_button.setStyleSheet("background-color: #3498db; color: white; border: none;")
            self.video_label.setStyleSheet("border: 2px solid black;")

    def toggle_theme(self, checked):
        self.dark_mode = checked
        self.update_theme()

    def change_camera(self):
        new_index = int(self.camera_combo.currentText())
        if self.cap:
            self.cap.release()
        self.cap = cv2.VideoCapture(new_index)
        if not self.cap.isOpened():
            print(f"Error: Could not open camera {new_index}")
            self.video_label.setText("Camera not plugged in")

    def get_current_joint_angles(self):
        if not self.base:
            return None
        try:
            joint_angles = self.base.GetMeasuredJointAngles()
            return [angle.value for angle in joint_angles.joint_angles]
        except Exception as e:
            print(f"Error getting joint angles: {e}")
            return None

    def toggle_autonomy(self):
        self.is_autonomous = not self.is_autonomous
        print(f"Autonomous mode: {'ON' if self.is_autonomous else 'OFF'}")
        if not self.is_autonomous:
            self.stop_robot()

    def autonomous_loop(self):
        while True:
            if not self.is_autonomous or not self.base:
                time.sleep(0.1)
                continue

            command = Base_pb2.TwistCommand()
            command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
            command.duration = 0
            twist = command.twist

            if self.last_tag_position and self.last_tag_pose:
                distance_cm = self.last_tag_position["distance"]
                rel_x = self.last_tag_position["rel_x"]
                rel_y = self.last_tag_position["rel_y"]
                distance_m = distance_cm / 100

                rel_x_m = rel_x * distance_m / FOCAL_LENGTH - CAMERA_OFFSET_X
                rel_y_m = rel_y * distance_m / FOCAL_LENGTH - CAMERA_OFFSET_Y
                delta_z_m = distance_m - (TARGET_DISTANCE_CM / 100)

                if (abs(rel_x) > CENTER_TOLERANCE or 
                    abs(rel_y) > CENTER_TOLERANCE or 
                    abs(delta_z_m) > 0.01):
                    twist.linear_x = SPEED_FACTOR * rel_x_m
                    twist.linear_y = SPEED_FACTOR * -rel_y_m
                    twist.linear_z = SPEED_FACTOR * delta_z_m

                rvec = self.last_tag_pose["rvec"]
                R, _ = cv2.Rodrigues(rvec)
                tag_z_axis = R[:, 2]
                tool_z_axis = np.array([0, 0, 1])
                desired_z_axis = -tag_z_axis
                error_vector = np.cross(tool_z_axis, desired_z_axis)
                error_magnitude = np.linalg.norm(error_vector)

                if error_magnitude > DEADBAND:
                    error_vector /= error_magnitude
                    twist.angular_x = max(min(P_GAIN * error_vector[0], MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)
                    twist.angular_y = max(min(P_GAIN * error_vector[1], MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)
                    twist.angular_z = max(min(P_GAIN * error_vector[2], MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

                sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
                singular = sy < 1e-6
                if not singular:
                    roll = math.degrees(math.atan2(R[2,1], R[2,2]))
                    pitch = math.degrees(math.atan2(-R[2,0], sy))
                    yaw = math.degrees(math.atan2(R[1,0], R[0,0]))
                else:
                    roll = math.degrees(math.atan2(-R[1,2], R[1,1]))
                    pitch = math.degrees(math.atan2(-R[2,0], sy))
                    yaw = 0

                print(f"Aligning: Distance={distance_cm:.1f}cm, "
                      f"x={twist.linear_x:.3f}, y={twist.linear_y:.3f}, z={twist.linear_z:.3f}, "
                      f"roll={twist.angular_x:.3f}, pitch={twist.angular_y:.3f}, yaw={twist.angular_z:.3f}, "
                      f"error={error_magnitude:.3f}, R={roll:.1f}, P={pitch:.1f}, Y={yaw:.1f}, "
                      f"err_x={error_vector[0]:.3f}, err_y={error_vector[1]:.3f}, err_z={error_vector[2]:.3f}")

                if (abs(rel_x) <= CENTER_TOLERANCE and 
                    abs(rel_y) <= CENTER_TOLERANCE and 
                    abs(delta_z_m) <= 0.01 and 
                    error_magnitude < DEADBAND):
                    print("Aligned with tag")
                    twist.linear_x = twist.linear_y = twist.linear_z = 0
                    twist.angular_x = twist.angular_y = twist.angular_z = 0

            elif time.time() - self.last_tag_time > SEARCH_TIMEOUT:
                self.search_angle += 0.3
                twist.linear_x = SEARCH_SPEED * math.cos(self.search_angle)
                twist.linear_y = SEARCH_SPEED * math.sin(self.search_angle)
                twist.linear_z = 0
                print(f"Searching: x={twist.linear_x:.3f}, y={twist.linear_y:.3f}")

            if any([twist.linear_x, twist.linear_y, twist.linear_z, 
                    twist.angular_x, twist.angular_y, twist.angular_z]):
                self.base.SendTwistCommand(command)
            else:
                self.base.Stop()

            time.sleep(0.1)

    def reset_robot(self):
        if not self.base or not self.initial_joint_angles:
            print("Cannot reset: No connection or initial angles")
            return

        if self.is_autonomous:
            self.is_autonomous = False
            self.stop_robot()
            time.sleep(1)

        def reset():
            try:
                self.base.ClearFaults()
                time.sleep(0.5)
                servo_mode = Base_pb2.ServoingModeInformation()
                servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
                self.base.SetServoingMode(servo_mode)
                time.sleep(1)

                start_time = time.time()
                while time.time() - start_time < TIMEOUT_DURATION:
                    current_angles = self.get_current_joint_angles()
                    if not current_angles:
                        print("Failed to get joint angles")
                        break

                    joint_speeds = Base_pb2.JointSpeeds()
                    done = True
                    for i, (current, target) in enumerate(zip(current_angles, self.initial_joint_angles)):
                        error = target - current
                        while error > 180: error -= 360
                        while error <= -180: error += 360
                        if abs(error) > JOINT_TOLERANCE:
                            done = False
                            speed = max(min(error * 0.1, JOINT_SPEED), -JOINT_SPEED)
                            joint_speed = joint_speeds.joint_speeds.add()
                            joint_speed.joint_identifier = i
                            joint_speed.value = speed
                        else:
                            joint_speed = joint_speeds.joint_speeds.add()
                            joint_speed.joint_identifier = i
                            joint_speed.value = 0.0

                    if done:
                        print("Reset complete")
                        break

                    self.base.SendJointSpeedsCommand(joint_speeds)
                    time.sleep(0.1)

                self.base.Stop()
            except Exception as e:
                print(f"Reset failed: {e}")
                self.base.Stop()

        threading.Thread(target=reset, daemon=True).start()

    def stop_robot(self):
        if self.base:
            try:
                self.base.Stop()
                print("Robot stopped")
            except Exception as e:
                print(f"Stop failed: {e}")

    def update_frame(self):
        if not self.cap.isOpened():
            self.video_label.setText("Camera not plugged in")
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = frame.shape[:2]
        center_frame = (width // 2, height // 2)

        cv2.line(frame, (0, center_frame[1]), (width, center_frame[1]), (0,255,255), 2)
        cv2.line(frame, (center_frame[0], 0), (center_frame[0], height), (0,255,255), 2)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            max_area = 0
            for i, corner in enumerate(corners):
                corner = corner.reshape((4, 2))
                marker_center = corner.mean(axis=0).astype(int)
                rel_x = marker_center[0] - center_frame[0]
                rel_y = center_frame[1] - marker_center[1]
                area = cv2.contourArea(corner)

                if area > max_area:
                    max_area = area
                    width_pixels = (np.linalg.norm(corner[0] - corner[1]) + 
                                   np.linalg.norm(corner[1] - corner[2])) / 2
                    distance_m = (REAL_TAG_WIDTH * FOCAL_LENGTH) / width_pixels
                    distance_cm = distance_m * 100
                    self.last_tag_position = {"rel_x": rel_x, "rel_y": rel_y, "distance": distance_cm}

                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corner], REAL_TAG_WIDTH, CAMERA_MATRIX, DIST_COEFFS
                    )
                    self.last_tag_pose = {"rvec": rvec[0][0], "tvec": tvec[0][0]}

                    R, _ = cv2.Rodrigues(rvec[0][0])
                    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
                    singular = sy < 1e-6
                    if not singular:
                        roll = math.degrees(math.atan2(R[2,1], R[2,2]))
                        pitch = math.degrees(math.atan2(-R[2,0], sy))
                        yaw = math.degrees(math.atan2(R[1,0], R[0,0]))
                    else:
                        roll = math.degrees(math.atan2(-R[1,2], R[1,1]))
                        pitch = math.degrees(math.atan2(-R[2,0], sy))
                        yaw = 0
                    cv2.putText(frame, f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}",
                                (marker_center[0], marker_center[1]+40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                cv2.polylines(frame, [corner.astype(int)], True, (0,255,0), 2)
                cv2.circle(frame, tuple(marker_center), 5, (0,0,255), -1)
                text = f"{self.tag_info.get(int(ids[i][0]), {}).get('seed', 'Unknown')} ({rel_x}, {rel_y})"
                cv2.putText(frame, text, (marker_center[0], marker_center[1]+20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

            self.distance_label.setText(f"Distance: {distance_cm:.1f} cm")
            self.last_tag_time = time.time()
        else:
            self.distance_label.setText("Distance: N/A")
            self.last_tag_position = None
            self.last_tag_pose = None

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(frame.data, width, height, frame.strides[0], QImage.Format_RGB888)
        self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def close_app(self):
        self.timer.stop()
        if self.cap:
            self.cap.release()
        if hasattr(self, 'device_connection'):
            self.device_connection.__exit__(None, None, None)
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AprilTagApp()
    window.show()
    sys.exit(app.exec_())