import cv2
import numpy as np
import apriltag
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting
import json

# Load camera calibration data
with open('cameraMatrix.pkl', 'rb') as f:
    camera_matrix = pickle.load(f)

with open('dist.pkl', 'rb') as f:
    dist_coeffs = pickle.load(f)

cap = cv2.VideoCapture(0)  # Change index if needed

detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
tag_size = 0.079  # Tag size in meters

reference_tag_id = 0  # Use AprilTag ID 0 as the reference (origin)
tvec_0 = None
R_0 = None
center_0 = None

# Variable to control data collection (toggles on/off with 's')
collect_data = False

# List to store the historical positions for JSON export
trajectory = []

# Smoothing parameter for low-pass filter (lower values yield smoother but slower updates)
smoothing_factor = 0.2
smoothed_pos = None  # Will hold the filtered relative position

# Flag to decide if we need to plot after quitting
plot_final = False

print("Instructions:")
print(" - Press 's' to toggle data collection ON/OFF.")
print(" - Press 'A' to quit capture and plot the saved trajectory data.")
print(" - Press 'q' to quit without plotting.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale and detect AprilTags
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)
    centers = {}  # To hold center points for drawing

    for detection in detections:
        tag_id = detection.tag_id

        # Define the 3D coordinates of the AprilTag corners (object points)
        object_points = np.array([
            [-tag_size / 2,  tag_size / 2, 0],
            [ tag_size / 2,  tag_size / 2, 0],
            [ tag_size / 2, -tag_size / 2, 0],
            [-tag_size / 2, -tag_size / 2, 0]
        ], dtype=np.float32)

        # Get detected 2D image points for the tag corners
        image_points = np.array(detection.corners, dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if success:
            # Convert rotation vector to a matrix
            R, _ = cv2.Rodrigues(rvec)
            center_x = int(np.mean(image_points[:, 0]))
            center_y = int(np.mean(image_points[:, 1]))
            centers[tag_id] = (center_x, center_y)

            # Set the reference tag data when detecting tag ID 0
            if tag_id == reference_tag_id:
                tvec_0 = tvec
                R_0 = R
                center_0 = (center_x, center_y)
                cv2.putText(frame, "Reference set", (center_x, center_y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # For non-reference tags, compute the relative translation vector
            if tvec_0 is not None and R_0 is not None and tag_id != reference_tag_id:
                # Compute relative translation vector from tag 0
                relative_tvec = np.matmul(R_0.T, (tvec - tvec_0))
                # Apply a fixed offset (in meters) to shift the coordinate system
                # Example: if you want the reference to shift 10 cm to the left (i.e., tag0 appears 10 cm to the right),
                # you may set offset_x = -10 (remember to convert later since we work in centimeters here)
                # In this example, we add an offset vector in centimeters (adjust these values to suit your setup).
                offset_cm = np.array([[ -45.0 ], [ 0.0 ], [ 0.0 ]])  # shift X by -10 cm
                relative_tvec = relative_tvec + (offset_cm / 100.0)  # convert offset to meters

                pos_cm = relative_tvec * 100  # Convert from meters to centimeters

                # [The rest of your smoothing/collection code follows...]


                # If collection is enabled, apply smoothing and save the data point
                if collect_data:
                    if smoothed_pos is None:
                        smoothed_pos = pos_cm.ravel()
                    else:
                        smoothed_pos = (1 - smoothing_factor) * smoothed_pos + smoothing_factor * pos_cm.ravel()
                    current_pos = smoothed_pos
                    trajectory.append(current_pos.tolist())
                else:
                    current_pos = pos_cm.ravel()

                # Display the relative position on the video frame
                text = f"ID {tag_id}: X:{current_pos[0]:.1f}cm Y:{current_pos[1]:.1f}cm Z:{current_pos[2]:.1f}cm"
                cv2.putText(frame, text, (center_x, center_y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Draw the tag boundaries
        for i in range(4):
            pt1 = tuple(map(int, detection.corners[i]))
            pt2 = tuple(map(int, detection.corners[(i + 1) % 4]))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

    # Draw a line from the reference tag to other detected tags, if available
    if center_0 is not None:
        for tag_id, center in centers.items():
            if tag_id != reference_tag_id:
                cv2.line(frame, center_0, center, (255, 0, 0), 2)

    # Display an indicator on the frame if data collection is active
    if collect_data:
        cv2.putText(frame, "Collecting Data", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("AprilTag Detection", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # Toggle data collection on/off when 's' is pressed
        collect_data = not collect_data
        print("Data collection toggled. Now collecting:", collect_data)
    elif key == ord('A'):
        # Set flag for final plotting and break out of the loop
        plot_final = True
        break
    elif key == ord('q'):
        # Quit immediately without plotting final trajectory
        break

cap.release()
cv2.destroyAllWindows()

# Save the trajectory data to a JSON file
with open('trajectory_data.json', 'w') as f:
    json.dump(trajectory, f)
print("Trajectory data saved to trajectory_data.json")

# If the user pressed 'A', then plot the saved trajectory data
if plot_final:
    # Load the trajectory data
    with open('trajectory_data.json', 'r') as f:
        traj_data = json.load(f)
    traj_data = np.array(traj_data)
    if traj_data.size == 0:
        print("No trajectory data to plot!")
    else:
        # Convert from centimeters to meters for plotting
        traj_m = traj_data / 100.0
        
        # Extract x, y, and z coordinates
        x = traj_m[:, 0]
        y = traj_m[:, 1]
        z = traj_m[:, 2]

        # Create a 3D plot with the specified axis limits
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(x, y, z, marker='o', linestyle='-')
        
        # Set axis limits as specified (in meters)
        ax.set_xlim(-0.52, 0.52)
        ax.set_ylim(-0.22, 0.22)
        ax.set_zlim(0, 0.60)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("3D Trajectory Data")
        
        plt.show()
