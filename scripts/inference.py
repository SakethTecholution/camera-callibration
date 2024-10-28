"""
@Author: Saketh
This code for inference the live camera pose and predict/move the robot pose 
Its collects the 10 samples and average the pose and move the robot to that pose

NOTE:- X,Y,Z are taken from pixelto3d using realsense intrinsics for better accuracy 

"""
import numpy as np
from spatialmath.base import tr2eul
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
from CobotClient import CobotClient
import time
import cv2
from SimpleHandEye.interfaces.apriltag import ApriltagTracker

# Calibration matrices remain unchanged
X = np.array([
    [0.979, 0.035, 0.202, -0.132],
    [-0.035, 0.999, -0.002, 0.002],
    [-0.202, -0.005, 0.979, 0.099],
    [0.0, 0.0, 0.0, 1.0]
])

Y = np.array([
    [-0.467, 0.598, -0.652, 0.657],
    [0.88, 0.388, -0.274, -0.318],
    [0.089, -0.701, -0.707, 1.018], 
    [0.0, 0.0, 0.0, 1.0]
])

def average_poses(pose_matrices):
    """Average multiple 4x4 transformation matrices."""
    translations = np.array([pose[:3, 3] for pose in pose_matrices])
    rotations = np.array([pose[:3, :3] for pose in pose_matrices])
    
    avg_translation = np.mean(translations, axis=0)
    
    r = R.from_matrix(rotations)
    quats = r.as_quat()
    avg_quat = np.mean(quats, axis=0)
    avg_quat = avg_quat / np.linalg.norm(avg_quat)
    avg_rotation = R.from_quat(avg_quat).as_matrix()
    
    avg_pose = np.eye(4)
    avg_pose[:3, :3] = avg_rotation
    avg_pose[:3, 3] = avg_translation
    
    return avg_pose

def initialize_realsense():
    """Initialize RealSense camera pipeline and configure streams."""
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    align_to = rs.align(rs.stream.color)
    
    color_profile = profile.get_stream(rs.stream.color)
    color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
    
    K = np.array([
        [color_intrinsics.fx, 0, color_intrinsics.ppx],
        [0, color_intrinsics.fy, color_intrinsics.ppy],
        [0, 0, 1]
    ])
    
    D = np.array(color_intrinsics.coeffs)
    
    return pipeline, align_to, depth_scale, K, D

def get_xyz_from_pixel(depth_frame, pixel, depth_scale, depth_intrinsics):
    """Get 3D coordinates from pixel coordinates using depth frame."""
    try:
        u, v = int(pixel[0]), int(pixel[1])
        depth = np.asanyarray(depth_frame.get_data())
        depth_value = depth[v, u] * depth_scale
        
        if depth_value <= 0 or depth_value > 10:
            print("Invalid depth value at pixel")
            return None
            
        point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth_value)
        return np.array(point)
        
    except Exception as e:
        print(f"Error getting 3D coordinates: {e}")
        return None

def test_getHandPose(robot_pose):
    """Extract translation in mm and rotation angles in degrees."""
    x, y, z = robot_pose[:3, 3] * 1000  # Convert to mm
    R_matrix = robot_pose[:3, :3]
    a, b, c = tr2eul(R_matrix, flip=True, unit='deg')
    return x, y, z, a, b, c

def drawPoseAxis(frame, pose, K, tag_size):
    """Draw coordinate axes for AprilTag pose."""
    axis_length = tag_size / 2
    axis_points = np.float32([[0, 0, 0], [axis_length, 0, 0],
                             [0, axis_length, 0], [0, 0, axis_length]])
    
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    
    img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, K, None)
    img_points = img_points.reshape(-1, 2).astype(int)
    
    origin = tuple(img_points[0])
    cv2.line(frame, origin, tuple(img_points[1]), (0, 0, 255), 2)
    cv2.line(frame, origin, tuple(img_points[2]), (0, 255, 0), 2)
    cv2.line(frame, origin, tuple(img_points[3]), (255, 0, 0), 2)

def main():
    # Initialize systems
    pipeline, align, depth_scale, K, D = initialize_realsense()
    tag_tracker = ApriltagTracker(
        tag_size=0.07,
        intrinsic_matrix=K,
        distortion_coeffs=D,
        family='tag36h11'
    )
    robot = CobotClient()
    
    # List to store B matrices
    B_matrices = []
    sample_count = 0
    num_samples = 10
    
    print("Collecting 10 frames for averaging...")
    
    try:
        while sample_count < num_samples:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()
            
            # Process frame with AprilTag tracker
            info = tag_tracker.getPoseAndCorners(color_image, tag_id=0)
            
            if info:
                B = info['pose']
                corners = info['corners']
                center = info['center']
                
                # Draw visualization
                cv2.polylines(color_image, [corners.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(color_image, tuple(center.astype(int)), 5, (255, 0, 0), -1)
                drawPoseAxis(color_image, B, K, tag_tracker.tag_size)
                
                # Get 3D position from center pixel
                translation = get_xyz_from_pixel(
                    depth_frame,
                    center,
                    depth_scale,
                    depth_intrinsics
                )
                
                if translation is not None:
                    # Update B matrix with accurate translation
                    B[:3, 3] = translation
                    B_matrices.append(B)
                    sample_count += 1
                    print(f"Collected sample {sample_count}/{num_samples}")
                    
                    # Display sample count
                    cv2.putText(color_image, f"Samples: {sample_count}/{num_samples}", 
                              (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display the frame
            cv2.imshow('RealSense', color_image)
            cv2.waitKey(1)
            
        # Close visualization window after collecting samples
        cv2.destroyAllWindows()
        
        # Process collected samples
        print("\nProcessing collected samples...")
        avg_B = average_poses(B_matrices)
        
        # Calculate final robot pose
        A = Y @ avg_B @ np.linalg.inv(X)
        x, y, z, a, b, c = test_getHandPose(A)
        
        print("\nSending averaged pose to robot:")
        print(f"X={x:.2f}mm, Y={y:.2f}mm, Z={z:.2f}mm")
        print(f"A={a:.2f}°, B={b:.2f}°, C={c:.2f}°")
        
        # Move robot to predicted pose
        z = z + 35 # Adding 25mm for verfiction and not hitting the object while it is on ground 
        robot.move_to_pose(x, y, z, a, b, c)
        print("\nMotion complete!")
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()