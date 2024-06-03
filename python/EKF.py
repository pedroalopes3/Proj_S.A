import cv2
import rosbag
from cv_bridge import CvBridge
import os
import numpy as np

N_landmarks = 8
n_state = 3

R = np.diag([0.002, 0.002, 0.0005])  # Parameter of motion uncertainty
Q = np.diag([0.003, 0.005])  # Parameter of measurement uncertainty

# EKF Estimation Variables
miu = np.zeros((n_state + 2 * N_landmarks, 1))  # State estimate (robot pose and landmark positions)
sigma = np.zeros((n_state + 2 * N_landmarks, n_state + 2 * N_landmarks))  # State uncertainty, covariance matrix

miu[:] = np.nan  # Initialize state estimate with nan values
np.fill_diagonal(sigma, 100)  # Initialize state uncertainty with large variances, no correlations

Fx = np.block([[np.eye(3), np.zeros((3, 2 * N_landmarks))]])

# Initialize CvBridge
bridge = CvBridge()

# Absolute path to your ROS bag
bag_path = '/home/miguel/bag5.bag'  # Full path to your ROS bag file

# Check if the file exists
if not os.path.exists(bag_path):
    raise FileNotFoundError(f"The file {bag_path} does not exist.")

def compute_time_between_samples(bag_file, topic_name):
    # Open the bag file
    bag = rosbag.Bag(bag_file)
    
    # Initialize variables to store the previous timestamp and time differences
    previous_time = None
    time_diffs = []

    # Iterate through the messages in the specified topic
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        current_time = t.to_sec()
        
        if previous_time is not None:
            # Calculate the time difference (dt)
            dt = current_time - previous_time
            time_diffs.append(dt)
        
        # Update the previous timestamp
        previous_time = current_time

    bag.close()
    
    return time_diffs

def calculate_velocities(previous_pose, current_pose, dt):
    # Extract position from poses
    x1, y1 = previous_pose.position.x, previous_pose.position.y
    x2, y2 = current_pose.position.x, current_pose.position.y

    # Calculate true linear velocity (Euclidean distance)
    linear_velocity = np.sqrt((x2 - x1)**2 + (y2 - y1)**2) / dt

    # Extract orientations (assuming 2D plane, ignoring z, w)
    theta1 = np.arctan2(2 * (previous_pose.orientation.w * previous_pose.orientation.z),
                        1 - 2 * (previous_pose.orientation.z**2))
    theta2 = np.arctan2(2 * (current_pose.orientation.w * current_pose.orientation.z),
                        1 - 2 * (current_pose.orientation.z**2))

    # Calculate angular velocity
    angular_velocity = (theta2 - theta1) / dt

    return linear_velocity, angular_velocity

def prediction_step(miu, sigma, u, dt):
    x = miu[0]
    y = miu[1]
    theta = miu[2]
    v = u[0]
    w = u[1]

    states_m = np.zeros((n_state, 1))

    if abs(w) > 0.000001:
        states_m[0] = miu[0] + (-(v / w) * np.sin(theta)) + ((v / w) * np.sin(theta + w * dt))
        states_m[1] = miu[1] + ((v / w) * np.cos(theta)) + (-(v / w) * np.cos(theta + w * dt))
    else:
        states_m[0] = v * dt * np.cos(theta)
        states_m[1] = v * dt * np.sin(theta)

    states_m[2] = w * dt  # Update for robot heading theta

    miu = miu + np.matmul(np.transpose(Fx), states_m)  # Update state estimate, simple use model with current state estimate

    state_jacobian = np.zeros((3, 3))

    if abs(w) > 0.000001:
        state_jacobian[0, 2] = (v / w) * np.cos(theta) - (v / w) * np.cos(theta + w * dt)
        state_jacobian[1, 2] = (v / w) * np.sin(theta) - (v / w) * np.sin(theta + w * dt)
    else:
        state_jacobian[0, 2] = -v * np.sin(theta) * dt
        state_jacobian[1, 2] = v * np.cos(theta) * dt

    G = np.eye(sigma.shape[0]) + np.transpose(Fx).dot(state_jacobian).dot(Fx)  # How the model transforms uncertainty
    sigma = G.dot(sigma).dot(np.transpose(G)) + np.transpose(Fx).dot(R).dot(Fx)
    return miu, sigma




def update_step(miu, sigma, zs):
    x = miu[0]
    y = miu[1]
    theta = miu[2]
    delta_zs = [np.zeros((2, 1)) for _ in range(N_landmarks)]  # A list of how far an actual measurement is from the estimate measurement
    Ks = [np.zeros((miu.shape[0], 2)) for _ in range(N_landmarks)]  # A list of matrices stored for use outside the measurement for loop
    Hs = [np.zeros((2, miu.shape[0])) for _ in range(N_landmarks)]  # A list of matrices stored for use outside the measurement for loop

    for z in zs:
        dist, phi, lidx = z
        lidx = int(lidx)  # Ensure lidx is an integer
        miu_landmark = miu[n_state + lidx * 2: n_state + lidx * 2 + 2]  # Get the estimated position of the landmark
        if np.isnan(miu_landmark[0]):  # If the landmark hasn't been observed before, then initialize (lx,ly)
            miu_landmark[0] = x + dist * np.cos(phi + theta)
            miu_landmark[1] = y + dist * np.sin(phi + theta)
            miu[n_state + lidx * 2: n_state + lidx * 2 + 2] = miu_landmark  # Save these values to the state estimate mu

        delta = miu_landmark - np.array([[x], [y]], dtype=np.float64)  # Helper variable
        q = np.linalg.norm(delta)**2  # Helper variable

        dist_est = np.sqrt(q)  # Distance between robot estimate and and landmark estimate, i.e., distance estimate
        phi_est = np.arctan2(delta[1, 0], delta[0, 0]) - theta
        phi_est = np.arctan2(np.sin(phi_est), np.cos(phi_est))  # Estimated angle between robot heading and landmark
        z_est_arr = np.array([[dist_est], [phi_est]], dtype=np.float64)  # Estimated observation, in numpy array
        z_act_arr = np.array([[dist], [phi]], dtype=np.float64)  # Actual observation in numpy array
        delta_zs[lidx] = z_act_arr - z_est_arr  # Difference between actual and estimated observation

        # Helper matrices in computing the measurement update
        Fxj = np.block([[Fx], [np.zeros((2, Fx.shape[1]))]])
        Fxj[n_state: n_state + 2, n_state + 2 * lidx: n_state + 2 * lidx + 2] = np.eye(2)
        H = np.array([[-delta[0, 0] / np.sqrt(q), -delta[1, 0] / np.sqrt(q), 0, delta[0, 0] / np.sqrt(q), delta[1, 0] / np.sqrt(q)],
                      [delta[1, 0] / q, -delta[0, 0] / q, -1, -delta[1, 0] / q, delta[0, 0] / q]])
        H = H.dot(Fxj)

        Hs[lidx] = H  # Added to list of matrices
        
        # Ensure H and sigma have the correct data type
        H = H.astype(np.float64)
        sigma = sigma.astype(np.float64)

        try:
            Ks[lidx] = sigma.dot(np.transpose(H)).dot(np.linalg.inv(H.dot(sigma).dot(np.transpose(H)) + Q))  # Add to list of matrices
        except np.linalg.LinAlgError as e:
            print(f"LinAlgError for landmark {lidx}: {e}")
            continue

    # After storing appropriate matrices, perform measurement update of mu and sigma
    miu_offset = np.zeros(miu.shape, dtype=np.float64)  # Offset to be added to state estimate
    sigma_factor = np.eye(sigma.shape[0], dtype=np.float64)  # Factor to multiply state uncertainty
    for lidx in range(N_landmarks):
        miu_offset += Ks[lidx].dot(delta_zs[lidx])  # Compute full mu offset
        sigma_factor -= Ks[lidx].dot(Hs[lidx])  # Compute full sigma factor
    miu = miu + miu_offset  # Update state estimate
    sigma = sigma_factor.dot(sigma)  # Update state uncertainty

    return miu, sigma



# Compute time differences for /pose topic
pose_topic = '/pose'
time_diffs = compute_time_between_samples(bag_path, pose_topic)

if len(time_diffs) > 0:
    dt = np.mean(time_diffs)
else:
    dt = 0  # Default value in case no messages are found

print(f"Computed average dt: {dt}")

# Open the ROS bag
bag = rosbag.Bag(bag_path, 'r')

# Load the dictionary that was used to generate the markers
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters
parameters = cv2.aruco.DetectorParameters_create()

# Size of the ArUco marker (in meters)
marker_size = 0.075  # Assuming the marker size is 7.5 cm

# Assuming the camera calibration parameters are known:
# Focal length and sensor size can be obtained via camera calibration
focal_length = 525  # Example focal length in pixels

# Camera matrix and distortion coefficients
# Replace these with your actual calibration data
camera_matrix = np.array([[focal_length, 0, 320],
                          [0, focal_length, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))  # Assuming no lens distortion

# Variables to store the previous pose
previous_pose = None
# Process each frame
for topic, msg, t in bag.read_messages(topics=[pose_topic, '/camera/rgb/image_color']):
    if topic == pose_topic:
        # Calculate velocities if we have a previous pose
        if previous_pose is not None:
            current_pose = msg.pose.pose
            linear_velocity, angular_velocity = calculate_velocities(previous_pose, current_pose, dt)
            print(f"linear velocity: {linear_velocity:.2f}")
            print(f"Angular velocity: {angular_velocity:.2f}")

            # Perform the prediction step
            u = np.array([linear_velocity, angular_velocity])
            miu, sigma = prediction_step(miu, sigma, u, dt)

        # Update previous pose
        previous_pose = msg.pose.pose

    elif topic == '/camera/rgb/image_color':
        # Convert the ROS Image message to a format OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect the markers in the image
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # If markers are detected
        if ids is not None:
            # Draw detected markers
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # Calculate distance to the marker if at least one is detected
            if len(ids) >= 1:
                # Get the first marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
                
                if tvecs is not None and len(tvecs) > 0:
                    # Draw axis for the first marker
                    cv_image = cv2.aruco.drawAxis(cv_image, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], 0.1)

                    # Calculate distance from camera to marker
                    distance = np.linalg.norm(tvecs[0][0])
                    print(f"Detected marker IDs: {ids.flatten()}") 

                    print(f"Distance to marker: {distance:.2f} meters")

                    # Get the x, y location of the marker
                    x, y, z = tvecs[0][0]

                    # Calculate the angle relative to the robot's coordinate system
                    angle_rad = np.arccos(x / distance)
                    angle_deg = np.degrees(angle_rad) - 90
                    print(f"Angle relative to robot: {angle_deg:.2f} degrees")

                    # Collect measurements for the update step
                    zs = []
                    for i, marker_id in enumerate(ids.flatten()):
                        dist = np.linalg.norm(tvecs[i][0])
                        phi = np.arctan2(tvecs[i][0][1], tvecs[i][0][0])
                        lidx = int(marker_id % N_landmarks)  # Ensure lidx is an integer
                        zs.append((dist, phi, lidx))

                    # Perform the update step
                    miu, sigma = update_step(miu, sigma, zs)
                else:
                    print("Pose estimation failed. tvecs is None or empty.")

        # Display the frame with markers
        cv2.imshow('Frame', cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Close the ROS bag and OpenCV windows
bag.close()
cv2.destroyAllWindows()
