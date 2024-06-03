import cv2
import rosbag
from cv_bridge import CvBridge
import os
import numpy as np
from geometry_msgs.msg import Twist

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
bag_path = '/home/miguel/bigbag2.bag'  # Full path to your ROS bag file

# Check if the file exists
if not os.path.exists(bag_path):
    raise FileNotFoundError(f"The file {bag_path} does not exist.")



# Function to extract and print twist values from TwistWithCovariance message
def extract_twist_values(twist_with_covariance_msg):
    twist_msg = twist_with_covariance_msg.twist
    linear_twist_x = twist_msg.linear.x
    angular_twist_z = twist_msg.angular.z
    print(f"Linear twist (x): {linear_twist_x:.3f} m/s") #THIS IS LINAE VELOCITY
    print(f"Angular twist (z): {angular_twist_z:.3f} rad/s") #THIS IS ANGULAR VELOCITY

# Compute time differences for /pose topic
pose_topic = '/pose'


dt = 0.034  # Default value in case no messages are found

print(f"Computed average dt: {dt}")

# Open the ROS bag
bag = rosbag.Bag(bag_path, 'r')

# Load the dictionary that was used to generate the markers
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize the detector parameters
parameters = cv2.aruco.DetectorParameters_create()

# Size of the ArUco marker (in meters)
marker_size = 0.18  # Assuming the marker size is 15 cm

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
        extract_twist_values(msg.twist)  # Extract and print 

    if topic == '/camera/rgb/image_color':
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
                    print(f"Detected marker IDs: {ids.flatten()}") #THIS IS THE MARKERS ID

                    print(f"Distance to marker: {distance:.2f} meters") #THISIS THE RANGE TO LANDMARK

                    x, y, z = tvecs[0][0]
                    # Calculate the distance (optional, but useful for understanding the context)
                    distance = np.sqrt(x**2 + y**2 + z**2)

                    # Calculate the angle relative to the robot's coordinate system in radians
                    angle_rad = np.arctan2(x, z)

                    # Convert the angle to degrees
                    angle_deg= np.degrees(angle_rad)
                    print(f"Angle relative to robot: {angle_deg:.2f} degrees")  #THIS IS THE BEARING OF THE LANDMARK 
                else:
                    print("Pose estimation failed. tvecs is None or empty.")

        # Display the frame with markers
        cv2.imshow('Frame', cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Close the ROS bag and OpenCV windows
bag.close()
cv2.destroyAllWindows()