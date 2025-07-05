import cv2
import numpy as np
import sqlite3

# Initialize the AprilTag detector using OpenCV
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Open the camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Function to get tag data from the database
def get_tag_info(tag_id):
    try:
        tag_id = int(tag_id)  # Ensure the tag ID is an integer
        print(f"Looking for Tag ID in database: {tag_id}")  # Debugging step
        conn = sqlite3.connect("tags.db")
        cursor = conn.cursor()
        cursor.execute("SELECT seed_type, x, y, z FROM tags WHERE id = ?", (tag_id,))
        result = cursor.fetchone()
        conn.close()
        
        if result:
            seed_type, x, y, z = result
            return f"ID: {tag_id}\nSeed: {seed_type}\nX: {x}, Y: {y}, Z: {z}"
        
        print(f"Tag {tag_id} not found in database!")  # Debugging step
        return f"Unknown Tag (ID: {tag_id})"
    except ValueError:
        print("Error: Tag ID is not a valid integer.")
        return "Unknown Tag"

# Dictionary to store tag detections across frames
tag_history = {}

# Number of consecutive frames a tag must appear to be considered valid
frame_threshold = 15

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags in the current frame
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        detected_ids = ids.flatten().astype(int)
        print(f"Detected Tag IDs: {detected_ids}")  # Debugging detected IDs

        current_tag_ids = set(detected_ids)

        # Update tag history: increment detection count or reset if not detected
        for tag_id in list(tag_history.keys()):
            if tag_id not in current_tag_ids:
                tag_history[tag_id] = 0
            else:
                tag_history[tag_id] += 1

        # Add new tags to the history, initialize with a count of 1
        for tag_id in current_tag_ids:
            if tag_id not in tag_history:
                tag_history[tag_id] = 1

        # Loop through detections and draw them on the frame, only for valid tags
        for i, tag_id in enumerate(detected_ids):
            if tag_history.get(tag_id, 0) >= frame_threshold:
                tag_corners = corners[i].astype(int).reshape(-1, 2)

                # Draw the bounding box around the detected tag
                for j in range(4):
                    cv2.line(frame, tuple(tag_corners[j]), tuple(tag_corners[(j + 1) % 4]), (0, 255, 0), 2)

                # Draw the center of the tag
                cX, cY = np.mean(tag_corners, axis=0).astype(int)
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

                # Retrieve tag information from the database
                tag_info = get_tag_info(tag_id)
                print(f"Tag ID {tag_id} Info: {tag_info}")  # Debug DB info

                # Display the information in a separate window
                info_window = np.ones((200, 400, 3), dtype=np.uint8) * 255
                y0, dy = 30, 30
                for i, line in enumerate(tag_info.split('\n')):
                    y = y0 + i * dy
                    cv2.putText(info_window, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                cv2.imshow('Tag Information', info_window)
    else:
        print("No tags detected.")  # Debugging step

    # Show the annotated frame
    cv2.imshow('Frame with AprilTags', frame)

    # Exit the loop if the 'q' key is pressed
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
