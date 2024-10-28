"""
@Author : B.Saketh 
This code for callibration EYE TO HAND/BASE Callibration 

A = Hand w.r.t Base
B = Tag w.r.t Camera
X = Tag w.r.t Hand
Y = Camera w.r.t Base

"""


import os
import csv
import cv2
import numpy as np
from SimpleHandEye.interfaces.apriltag import ApriltagTracker
from SimpleHandEye.interfaces.cameras import RealSenseCamera
from CobotClient import CobotClient
from scipy.spatial.transform import Rotation as R
from SimpleHandEye.solvers import OpenCVSolver
from spatialmath.base import eul2tr, tr2eul
import ast

np.set_printoptions(precision=3, suppress=True)
# Initialize RealSense camera
camera = RealSenseCamera(color_fps=30, depth_fps=30, enable_color=True, enable_depth=True)

# Initialize AprilTag tracker
intrinsics = camera.getIntrinsics()['RGB']
tag_tracker = ApriltagTracker(tag_size=0.06,  # 0.1m tag size
                              intrinsic_matrix=intrinsics['K'],
                              distortion_coeffs=intrinsics['D'],
                              family='tag36h11')

# Initialize robot client
robot = CobotClient()

# Initialize solver
solver = OpenCVSolver(type='AX=YB')

# Lists to store samples
A_samples = []
B_samples = []




def getHandPose():
    """Get robot pose and convert it to a homogeneous matrix."""
    robot_pose_vec = robot.get_current_pose()
    print("Raw Robot Pose:", robot_pose_vec)
    x = robot_pose_vec['X'] / 1000  # Convert to meters
    y = robot_pose_vec['Y'] / 1000
    z = robot_pose_vec['Z'] / 1000
    # Getting rotation matrix from ros service call 
    R_matrix = robot.get_current_rotm()
    print("Rotation Matrix:\n", R_matrix)
    t = np.array([[x], [y], [z]])
    H = np.block([
        [R_matrix, t],
        [np.zeros((1, 3)), np.array([[1]])]
    ])
    print("Converted Homogeneous Matrix (A):", H)
    return H, robot_pose_vec

def drawPoseAxis(frame, pose, K):
    """
    Draw the axis corresponding to the AprilTag's pose.
    """
    axis_length = tag_tracker.tag_size / 2
    axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0], 
                              [0, axis_length, 0], [0, 0, axis_length]])
    img_points, _ = cv2.projectPoints(axis_points, pose[:3, :3], pose[:3, 3], 
                                      K, None)
    img_points = img_points.reshape(-1, 2).astype(int)

    cv2.line(frame, tuple(img_points[0]), tuple(img_points[1]), (0, 0, 255), 2)  # X-axis (red)
    cv2.line(frame, tuple(img_points[0]), tuple(img_points[2]), (0, 255, 0), 2)  # Y-axis (green)
    cv2.line(frame, tuple(img_points[0]), tuple(img_points[3]), (255, 0, 0), 2)  # Z-axis (blue)

def get_tag_3d_pose(tag_info):
    """
    Extract the 3D pose from the tag information.
    """
    if tag_info is not None and 'pose' in tag_info:
        return tag_info['pose']
    return None

def save_sample():
    """
    Save the current sample to the sample lists in a  CSV file.
    """
    A, robot_pose_vec = getHandPose()
    A_samples.append(A)
    B_samples.append(B)
    # camera.grab_frames()
    # color_frame = camera.color_frame

    # cv2.imwrite("output_folder/" +f"sample_{len(A_samples)}.png", color_frame)
    
    # Print the robot's pose in terms of x, y, z, a, b, c
    print(f"Robot Pose: X: {robot_pose_vec['X']}, Y: {robot_pose_vec['Y']}, Z: {robot_pose_vec['Z']}, A: {robot_pose_vec['A']}, B: {robot_pose_vec['B']}, C: {robot_pose_vec['C']}")
    
    print(f"Sample {len(A_samples)} saved")
    print("A (Base2Hand):", A)
    print("B (Tag2Camera):", B)
    
    # Call the function to save the data to CSV
    save_to_csv(A, B, robot_pose_vec)

def save_to_csv(A, B, robot_pose_vec):
    """
    Save the homogeneous matrices A and B, along with the robot's pose (x, y, z, a, b, c),
    to a CSV file in append mode, adding headers if the file is new.
    """
    file_exists = os.path.exists('calibration_data.csv')

    with open('calibration_data.csv', mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write headers only if the file is new
        if not file_exists:
            writer.writerow([
                'Robot Pose (X)', 'Robot Pose (Y)', 'Robot Pose (Z)', 
                'Robot Pose (A)', 'Robot Pose (B)', 'Robot Pose (C)',
                'Camera Pose (Homogeneous Matrix A)', 
                'Raw Camera Data (Homogeneous Matrix B)'
            ])

        # Write the robot pose and matrices A and B
        writer.writerow([
            robot_pose_vec['X'], robot_pose_vec['Y'], robot_pose_vec['Z'], 
            robot_pose_vec['A'], robot_pose_vec['B'], robot_pose_vec['C'],
            A.flatten().tolist(), B.flatten().tolist()
        ])
        
    print("Data saved to CSV")

    """
    Solving for AX = YB in which 
    A is the base to hand matrix
    B is the tag to camera matrix
    X is hand to Canera matrix 
    Y is base to tag matrix
    """

def solve_calibration():
    if len(A_samples) < 3:
        print("Need at least 3 samples for calibration")
        return
    
    result = solver.solve(A_samples, B_samples)
    """
    Result will be calculated X and Y matrix
    """
    print("Calibration result:")
    print(result)
    
def solve_calibration_csv():
    """ This can be used when with a stored data file """
    A = []
    B = []
    
    with open('calibration_data.csv', mode='r', newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            # Skip the header row
            if row[0] == 'Robot Pose (X)':
                continue
            # Parse and reshape matrices A and B from the saved CSV strings
            A = np.array(ast.literal_eval(row[6]), dtype=float).reshape(4, 4)
            B = np.array(ast.literal_eval(row[7]), dtype=float).reshape(4, 4)

            B_inv = np.linalg.inv(B)

            # Append to sample lists
            A_samples.append(A)
            B_samples.append(B)
    
    result = solver.solve(A_samples, B_samples)
    print(result)
    print(type(result))
    """
    Result will be calculated X and Y matrix
    """
    print("Calibration result:")
    print(result)

    X, Y = result[0], result[1]

    for i in range(len(A_samples)):
        print(np.linalg.norm((A_samples[i]@X@np.linalg.inv(Y@B_samples[i]))[:3,-1]))



# Main loop
while True:
    # Grab frames
    camera.grab_frames()
    color_frame = camera.color_frame



    # Process frame with AprilTag tracker
    info = tag_tracker.getPoseAndCorners(color_frame, tag_id=0)
    
    if info is not None:
        B = info['pose']
        corners = info['corners']
        center = info['center']

        # Draw tag outline
        cv2.polylines(color_frame, [corners.astype(int)], True, (0, 255, 0), 2)

        # Draw tag center
        cv2.circle(color_frame, tuple(center.astype(int)), 5, (255, 0, 0), -1)

        # Draw pose axis
        drawPoseAxis(color_frame, B, intrinsics['K'])

    # Display the frame
    cv2.imshow('RealSense', color_frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('s'):
        save_sample()
    elif key == ord('c'):
        solve_calibration()
    elif key == ord('f'):
        solve_calibration_csv()

# Clean up
cv2.destroyAllWindows()
camera.close()

# Developer Notes:
"""
TODO:
1. Add saving in pickle format 
2. Replay observations for automatic callibration 
3. Add reprojection error for better accuracy
"""