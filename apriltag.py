import cv2
import numpy as np
import sqlite3

# Initialize the AprilTag detector from OpenCV
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
aruco_params = cv2.aruco.DetectorParameters()

# Improve detection by adjusting adaptive thresholding parameters
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshWinSizeStep = 10

# Detector
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Open the camera
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

def get_tag_info(tag_id):
    """Fetch seed type and coordinates from the SQLite database."""
    conn = sqlite3.connect("tags.db")
    cursor = conn.cursor()
    cursor.execute("SELECT seed_type, x, y, z FROM tags WHERE id = ?", (tag_id,))
    result = cursor.fetchone()
    conn.close()
    if result:
        seed_type, x, y, z = result
        return f"Seed: {seed_type}, X: {x}, Y: {y}, Z: {z}"
    return "Unknown Tag"

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the current frame
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        for i, tag_id in enumerate(ids.flatten()):
            tag_corners = corners[i].astype(int).reshape(-1, 2)

            # Draw bounding box with different color if ID 584 is detected
            color = (0, 255, 255) if tag_id == 584 else (0, 255, 0)
            for j in range(4):
                cv2.line(frame, tuple(tag_corners[j]), tuple(tag_corners[(j + 1) % 4]), color, 2)

            # Draw the center of the tag
            cX, cY = np.mean(tag_corners, axis=0).astype(int)
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # Retrieve tag information from the database
            tag_info = get_tag_info(tag_id)

            # Annotate the tag with its ID and associated data
            
            cv2.putText(frame, tag_info, (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)

    # Show the annotated frame
    cv2.imshow('Frame with AprilTags', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
