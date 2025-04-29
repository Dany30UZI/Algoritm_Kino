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
import csv

# Kinova Kortex API imports
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

# Constants
REAL_TAG_WIDTH = 0.05  # in meters
FOCAL_LENGTH = 550     # in pixels (adjust with real calibration)
TARGET_DISTANCE_CM = 20  # Target stopping distance in cm
SPEED_FACTOR = 0.2    # Speed scaling factor for position
P_GAIN = 0.02          # Proportional gain for orientation (reduced from 0.05)
D_GAIN = 0.01          # Derivative gain for orientation
DEADBAND = 5.0         # Deadband in degrees to avoid small corrections
TIMEOUT_DURATION = 30  # Timeout for reset
STEP_TIMEOUT = 5       # Timeout per step
CSV_FILE = 'gen3_lite_poses.csv'  # CSV file with recorded poses

class AprilTagApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Apriltag")
        self.resize(1000, 600)
        self.dark_mode = True

        # Central widget and layout
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.hbox = QHBoxLayout(self.central_widget)

        # Video feed area
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.hbox.addWidget(self.video_label, stretch=3)

        # Control panel
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
        self.change_camera_button.setFixedSize(150, 40)
        self.change_camera_button.clicked.connect(self.change_camera)
        self.vbox.addWidget(self.change_camera_button)
        self.vbox.addSpacing(20)

        self.move_robot_button = QPushButton("Move Robot")
        self.move_robot_button.setFixedSize(150, 40)
        self.move_robot_button.clicked.connect(self.move_robot)
        self.vbox.addWidget(self.move_robot_button)
        self.vbox.addSpacing(20)

        self.follow_path_button = QPushButton("Follow Path")
        self.follow_path_button.setFixedSize(150, 40)
        self.follow_path_button.clicked.connect(self.follow_path)
        self.vbox.addWidget(self.follow_path_button)
        self.vbox.addSpacing(20)

        self.reset_robot_button = QPushButton("Reset Robot")
        self.reset_robot_button.setFixedSize(150, 40)
        self.reset_robot_button.clicked.connect(self.reset_robot)
        self.vbox.addWidget(self.reset_robot_button)
        self.vbox.addSpacing(20)

        self.stop_robot_button = QPushButton("Stop Robot")
        self.stop_robot_button.setFixedSize(150, 40)
        self.stop_robot_button.clicked.connect(self.stop_robot)
        self.vbox.addWidget(self.stop_robot_button)
        self.vbox.addSpacing(20)

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
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Validate Aruco setup
        try:
            test_dict = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
            test_params = aruco.DetectorParameters()
            test_detector = aruco.ArucoDetector(test_dict, test_params)
            print("Aruco module validation: Detector created successfully")
        except Exception as e:
            print(f"Aruco module validation failed: {e}")

        # Tag info database
        self.tag_info = {
            1: {"seed": "Porumb"}, 2: {"seed": "Orz"}, 3: {"seed": "Orez"},
            4: {"seed": "Grau"}, 5: {"seed": "Soia"}, 6: {"seed": "Adauga"},
            7: {"seed": "Adauga"}, 8: {"seed": "Adauga"}, 9: {"seed": "Adauga"},
            10: {"seed": "Adauga"}
        }

        # Store latest tag position, angle, and control variables
        self.last_tag_position = None
        self.last_tag_angle = None
        self.prev_error = 0.0  # For derivative control
        self.prev_time = time.time()
        self.initial_joint_angles = None
        self.is_moving = False
        self.stop_event = threading.Event()

        # Initialize Kinova connection and store initial position
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            import utilities
            args = utilities.parseConnectionArguments()
            self.device_connection = utilities.DeviceConnection.createTcpConnection(args)
            self.router = self.device_connection.__enter__()
            self.base = BaseClient(self.router)
            print("Connected to Kinova arm successfully")
            servo_mode = Base_pb2.ServoingModeInformation()
            servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servo_mode)
            time.sleep(1)
            self.initial_joint_angles = self.get_current_joint_angles()
            if self.initial_joint_angles is None or len(self.initial_joint_angles) == 0:
                raise Exception("Failed to retrieve initial joint angles")
            self.initial_joint_angles = [self.normalize_angle(angle) for angle in self.initial_joint_angles]
            print(f"Initial joint angles stored (normalized): {self.initial_joint_angles}")
        except Exception as e:
            print(f"Failed to connect to Kinova arm or get initial position: {e}")
            self.base = None
            self.initial_joint_angles = None

        # Timer for video feed
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(30)

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    def get_current_joint_angles(self):
        """Retrieve the current joint angles of the robot."""
        if not self.base:
            return None
        try:
            joint_angles = self.base.GetMeasuredJointAngles()
            return [angle.value for angle in joint_angles.joint_angles]
        except Exception as e:
            print(f"Error getting joint angles: {e}")
            return None

    def update_theme(self):
        if self.dark_mode:
            self.central_widget.setStyleSheet("background-color: #34495e;")
            self.control_panel.setStyleSheet("background-color: #2c3e50; border: 2px solid white; border-radius: 15px;")
            self.title_label.setStyleSheet("color: white; font-size: 18px; font-weight: bold;")
            self.theme_label.setStyleSheet("color: white; font-size: 16px;")
            self.camera_label.setStyleSheet("color: white; font-size: 16px; margin-right: 10px;")
            self.camera_combo.setStyleSheet("QComboBox { background-color: #2c3e50; color: white; border: 2px solid white; border-radius: 5px; padding: 5px; font-size: 16px; } QComboBox::drop-down { border: none; }")
            self.change_camera_button.setStyleSheet("QPushButton { background-color: #e67e22; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #d35400; } QPushButton:pressed { background-color: #c0392b; }")
            self.move_robot_button.setStyleSheet("QPushButton { background-color: #3498db; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #2980b9; } QPushButton:pressed { background-color: #1c638d; }")
            self.follow_path_button.setStyleSheet("QPushButton { background-color: #9b59b6; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #8e44ad; } QPushButton:pressed { background-color: #7d3c98; }")
            self.reset_robot_button.setStyleSheet("QPushButton { background-color: #f39c12; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #e67e22; } QPushButton:pressed { background-color: #d35400; }")
            self.stop_robot_button.setStyleSheet("QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #c0392b; } QPushButton:pressed { background-color: #a93226; }")
            self.close_button.setStyleSheet("QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #c0392b; } QPushButton:pressed { background-color: #a93226; }")
            self.video_label.setStyleSheet("background: transparent; border: 2px solid white; border-radius: 15px;")
            self.distance_label.setStyleSheet("color: white; font-size: 16px;")
            self.theme_switch.setChecked(True)
        else:
            self.central_widget.setStyleSheet("background-color: #bdc3c7;")
            self.control_panel.setStyleSheet("background-color: #ecf0f1; border: 2px solid black; border-radius: 15px;")
            self.title_label.setStyleSheet("color: black; font-size: 18px; font-weight: bold;")
            self.theme_label.setStyleSheet("color: black; font-size: 16px;")
            self.camera_label.setStyleSheet("color: black; font-size: 16px; margin-right: 10px;")
            self.camera_combo.setStyleSheet("QComboBox { background-color: #ecf0f1; color: black; border: 2px solid black; border-radius: 5px; padding: 5px; font-size: 16px; } QComboBox::drop-down { border: none; }")
            self.change_camera_button.setStyleSheet("QPushButton { background-color: #e67e22; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #d35400; } QPushButton:pressed { background-color: #c0392b; }")
            self.move_robot_button.setStyleSheet("QPushButton { background-color: #3498db; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #2980b9; } QPushButton:pressed { background-color: #1c638d; }")
            self.follow_path_button.setStyleSheet("QPushButton { background-color: #9b59b6; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #8e44ad; } QPushButton:pressed { background-color: #7d3c98; }")
            self.reset_robot_button.setStyleSheet("QPushButton { background-color: #f39c12; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #e67e22; } QPushButton:pressed { background-color: #d35400; }")
            self.stop_robot_button.setStyleSheet("QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #c0392b; } QPushButton:pressed { background-color: #a93226; }")
            self.close_button.setStyleSheet("QPushButton { background-color: #3498db; color: white; border: none; border-radius: 10px; font-size: 16px; font-weight: bold; } QPushButton:hover { background-color: #2980b9; } QPushButton:pressed { background-color: #1c638d; }")
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

    def move_robot(self):
        if not self.base:
            print("Kinova arm not connected. Cannot move robot.")
            return

        if not self.last_tag_position or self.last_tag_angle is None:
            print("No AprilTag detected or orientation unknown. Cannot move to target.")
            return

        if self.is_moving:
            print("Robot is already moving. Stop it first.")
            return

        self.is_moving = True
        self.stop_event.clear()
        self.prev_error = 0.0  # Reset previous error
        self.prev_time = time.time()

        def move_to_target():
            command = Base_pb2.TwistCommand()
            command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
            command.duration = 0
            twist = command.twist

            while not self.stop_event.is_set():
                if not self.last_tag_position or self.last_tag_angle is None:
                    print("Lost tag detection or orientation. Stopping robot.")
                    break

                distance_cm = self.last_tag_position["distance"]
                if distance_cm <= TARGET_DISTANCE_CM + 1:
                    print(f"Reached target distance: {distance_cm:.1f} cm")
                    break

                # Position control
                distance_m = distance_cm / 100
                rel_x_m = self.last_tag_position["rel_x"] * distance_m / FOCAL_LENGTH
                rel_y_m = self.last_tag_position["rel_y"] * distance_m / FOCAL_LENGTH
                delta_z_m = (distance_m - TARGET_DISTANCE_CM / 100)
                twist.linear_x = SPEED_FACTOR * rel_x_m
                twist.linear_y = SPEED_FACTOR * rel_y_m
                twist.linear_z = SPEED_FACTOR * delta_z_m

                # Orientation control (PD)
                current_time = time.time()
                dt = current_time - self.prev_time if current_time > self.prev_time else 0.1  # Avoid division by zero
                tag_angle = self.last_tag_angle
                error = self.normalize_angle(tag_angle)  # Target is 0°

                # Apply deadband
                if abs(error) < DEADBAND:
                    error = 0.0

                # Proportional term
                p_term = P_GAIN * error

                # Derivative term
                error_derivative = (error - self.prev_error) / dt
                d_term = D_GAIN * error_derivative

                # Total angular velocity
                twist.angular_z = -(p_term + d_term)  # Negative to correct towards 0°
                twist.angular_x = 0
                twist.angular_y = 0

                # Update previous values
                self.prev_error = error
                self.prev_time = current_time

                print(f"Moving: x={twist.linear_x:.3f}m, y={twist.linear_y:.3f}m, z={twist.linear_z:.3f}m, "
                      f"angular_z={twist.angular_z:.3f}rad/s, Distance={distance_cm:.1f}cm, Tag Angle={tag_angle:.1f}°, "
                      f"P={p_term:.3f}, D={d_term:.3f}")
                self.base.SendTwistCommand(command)
                time.sleep(0.1)

            self.base.Stop()
            time.sleep(1)
            print("Robot stopped.")
            self.is_moving = False

        threading.Thread(target=move_to_target, daemon=True).start()

    def follow_path(self):
        """Read CSV file and move the robot to each recorded pose."""
        if not self.base:
            print("Kinova arm not connected. Cannot follow path.")
            return

        if self.is_moving:
            print("Robot is already moving. Stop it first.")
            return

        self.is_moving = True
        self.stop_event.clear()

        def execute_path():
            # Load poses from CSV
            poses = []
            try:
                with open(CSV_FILE, 'r') as file:
                    reader = csv.DictReader(file)
                    for row in reader:
                        poses.append({
                            'Point_Number': int(row['Point_Number']),
                            'X': float(row['X']),
                            'Y': float(row['Y']),
                            'Z': float(row['Z']),
                            'Theta_X': float(row['Theta_X']),
                            'Theta_Y': float(row['Theta_Y']),
                            'Theta_Z': float(row['Theta_Z'])
                        })
            except FileNotFoundError:
                print(f"Error: {CSV_FILE} not found. Record poses first.")
                self.is_moving = False
                return
            except Exception as e:
                print(f"Error reading CSV: {e}")
                self.is_moving = False
                return

            if not poses:
                print("No poses found in CSV.")
                self.is_moving = False
                return

            print(f"Loaded {len(poses)} poses from {CSV_FILE}")

            # Move to each pose
            for pose in poses:
                if self.stop_event.is_set():
                    print("Path execution stopped by user.")
                    break

                action = Base_pb2.Action()
                action.name = f"Move to Point {pose['Point_Number']}"
                cartesian_pose = action.reach_pose.target_pose
                cartesian_pose.x = pose['X']
                cartesian_pose.y = pose['Y']
                cartesian_pose.z = pose['Z']
                cartesian_pose.theta_x = pose['Theta_X']
                cartesian_pose.theta_y = pose['Theta_Y']
                cartesian_pose.theta_z = pose['Theta_Z']

                print(f"Moving to Point {pose['Point_Number']}: "
                      f"X={pose['X']:.3f}, Y={pose['Y']:.3f}, Z={pose['Z']:.3f}, "
                      f"θx={pose['Theta_X']:.1f}°, θy={pose['Theta_Y']:.1f}°, θz={pose['Theta_Z']:.1f}°")

                try:
                    self.base.ExecuteAction(action)
                    time.sleep(2)  # Wait for movement to complete (adjust as needed)
                except Exception as e:
                    print(f"Error moving to Point {pose['Point_Number']}: {e}")
                    self.base.Stop()
                    break

            self.base.Stop()
            time.sleep(1)
            print("Path execution completed or stopped.")
            self.is_moving = False

        threading.Thread(target=execute_path, daemon=True).start()

    def reset_robot(self):
        if not self.base:
            print("Kinova arm not connected. Cannot reset robot.")
            return

        if not self.initial_joint_angles:
            print("Initial position not recorded. Cannot reset.")
            return

        if self.is_moving:
            print("Robot is moving. Stop it first before resetting.")
            return

        def reset_to_initial():
            try:
                self.base.ClearFaults()
                time.sleep(1)
                print("Faults cleared.")
            except Exception as e:
                print(f"Failed to clear faults: {e}")

            servo_mode = Base_pb2.ServoingModeInformation()
            servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(servo_mode)
            time.sleep(1)

            current_angles = self.get_current_joint_angles()
            if not current_angles:
                print("Failed to get current angles. Aborting reset.")
                return
            current_angles = [self.normalize_angle(angle) for angle in current_angles]
            print(f"Current joint angles before reset (normalized): {current_angles}")

            target_angles = [self.normalize_angle(angle) for angle in self.initial_joint_angles]
            print(f"Target joint angles: {target_angles}")

            steps = 5
            for step in range(1, steps + 1):
                intermediate_angles = [
                    current_angles[i] + (target_angles[i] - current_angles[i]) * step / steps
                    for i in range(len(current_angles))
                ]
                action = Base_pb2.Action()
                action.name = f"Reset Step {step}"
                joint_angles = action.reach_joint_angles.joint_angles
                for i, angle in enumerate(intermediate_angles):
                    joint_angle = joint_angles.joint_angles.add()
                    joint_angle.joint_identifier = i
                    joint_angle.value = angle

                print(f"Executing step {step}/{steps}: {intermediate_angles}")
                e = threading.Event()
                handle = self.base.OnNotificationActionTopic(
                    self.check_for_end_or_abort(e),
                    Base_pb2.NotificationOptions()
                )
                try:
                    self.base.ExecuteAction(action)
                    finished = e.wait(STEP_TIMEOUT)
                    self.base.Unsubscribe(handle)
                    if not finished:
                        print(f"Step {step} timed out.")
                        break
                    current_angles = self.get_current_joint_angles()
                    if not current_angles:
                        print("Failed to get angles after step. Aborting.")
                        break
                    current_angles = [self.normalize_angle(angle) for angle in current_angles]
                    print(f"After step {step}: {current_angles}")
                except Exception as e:
                    print(f"Step {step} failed: {e}")
                    self.base.Stop()
                    break

            final_angles = self.get_current_joint_angles()
            if final_angles and all(abs(self.normalize_angle(final_angles[i]) - target_angles[i]) < 1.0 for i in range(len(final_angles))):
                print(f"Reset complete. Final joint angles: {final_angles}")
            else:
                print(f"Reset may have failed. Final joint angles: {final_angles} vs Target: {target_angles}")

        threading.Thread(target=reset_to_initial, daemon=True).start()

    def stop_robot(self):
        if not self.base:
            print("Kinova arm not connected. Cannot stop robot.")
            return

        if not self.is_moving:
            print("Robot is not moving.")
            return

        try:
            self.stop_event.set()
            self.base.Stop()
            time.sleep(1)
            print("Robot stopped by user. Move script cancelled.")
            self.is_moving = False
        except Exception as e:
            print(f"Failed to stop robot: {e}")
            self.is_moving = False

    def check_for_end_or_abort(self, e):
        def check(notification, e=e):
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            print(f"EVENT: {event_name}")
            if notification.action_event == Base_pb2.ACTION_END:
                e.set()
            elif notification.action_event == Base_pb2.ACTION_ABORT:
                print(f"Action aborted. Reason code: {notification.abort_details}")
                e.set()
        return check

    def update_frame(self):
        if not self.cap.isOpened():
            self.video_label.setText("Camera not plugged in")
            return

        ret, frame = self.cap.read()
        if ret:
            filtered = cv2.bilateralFilter(frame, d=5, sigmaColor=75, sigmaSpace=75)
            gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
            height, width = frame.shape[:2]
            center_frame = (width // 2, height // 2)

            cv2.arrowedLine(frame, (0, center_frame[1]), (width, center_frame[1]), (0,255,255), 3, tipLength=0.05)
            cv2.arrowedLine(frame, (center_frame[0], height), (center_frame[0], 0), (0,255,255), 3, tipLength=0.05)
            cv2.putText(frame, "X", (width - 40, center_frame[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            cv2.putText(frame, "Y", (center_frame[0] + 10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
            cv2.circle(frame, center_frame, 5, (0,255,255), -1)

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
                        d1 = np.linalg.norm(corner[0] - corner[1])
                        d2 = np.linalg.norm(corner[1] - corner[2])
                        measured_width = (d1 + d2) / 2.0
                        distance = (REAL_TAG_WIDTH * FOCAL_LENGTH) / measured_width
                        distance_cm = distance * 100
                        self.last_tag_position = {"rel_x": rel_x, "rel_y": rel_y, "distance": distance_cm}

                        # Calculate the angle of the bottom edge (corner 0 to 1)
                        dx = corner[1][0] - corner[0][0]
                        dy = corner[1][1] - corner[0][1]
                        angle = math.degrees(math.atan2(dy, dx))
                        self.last_tag_angle = angle
                        cv2.putText(frame, f"Angle: {angle:.1f}°", (marker_center[0]-20, marker_center[1]+40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                    cv2.polylines(frame, [corner.astype(int)], isClosed=True, color=(0,255,0), thickness=3)
                    cv2.circle(frame, tuple(marker_center), 5, (0,0,255), -1)
                    info_text = f"{self.tag_info.get(int(ids[i][0]), {}).get('seed', 'Unknown')} | ({rel_x}, {rel_y})"
                    cv2.putText(frame, info_text, (marker_center[0]-20, marker_center[1]+20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

                self.distance_label.setText(f"Distance: {distance_cm:.1f} cm")
            else:
                self.distance_label.setText("Distance: N/A")
                self.last_tag_position = None
                self.last_tag_angle = None

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            qimg = QImage(frame.data, width, height, 3 * width, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def close_app(self):
        self.timer.stop()
        if self.cap is not None:
            self.cap.release()
        if hasattr(self, 'device_connection'):
            self.device_connection.__exit__(None, None, None)
        self.close()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AprilTagApp()
    window.show()
    sys.exit(app.exec_())