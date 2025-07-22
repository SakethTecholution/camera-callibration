import numpy as np
from spatialmath.base import tr2eul
import pyrealsense2 as rs

# from CobotClient import CobotClient
# from dsr_control_api.cobotclient import CobotClient
import time
import cv2
from ultralytics import YOLO
# import torch
import requests


X = np.array([
    [ 0.001, -0.053,  0.999, -0.038],
    [-1.0,   -0.024,  0.0,   -0.006],
    [ 0.024, -0.998, -0.053,  0.128],
    [ 0.0,    0.0,    0.0,    1.0  ]
])

Y = np.array([
    [-0.3,    0.496, -0.815,  0.525],
    [ 0.954,  0.161, -0.253, -0.396],
    [ 0.006, -0.853, -0.521,  0.729],
    [ 0.0,    0.0,    0.0,    1.0  ]
])

def average_poses(pose_matrices):
    """Average multiple 4x4 transformation matrices."""
    translations = np.array([pose[:3, 3] for pose in pose_matrices])
    rotations = np.array([pose[:3, :3] for pose in pose_matrices])

    avg_translation = np.mean(translations, axis=0)
    avg_rotation = np.mean(rotations, axis=0)

    avg_pose = np.eye(4)
    avg_pose[:3, :3] = avg_rotation
    avg_pose[:3, 3] = avg_translation

    return avg_pose


def initialize_realsense():
    """Initialize RealSense camera pipeline and configure streams."""
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    align_to = rs.align(rs.stream.color)

    return pipeline, align_to, profile


def get_xyz_from_pixel(depth_frame, pixel, depth_intrinsics):
    """Get 3D coordinates from pixel coordinates using depth frame."""
    try:
        u, v = int(pixel[0]), int(pixel[1])
        depth = depth_frame.get_distance(u, v)

        if depth <= 0 or depth > 2:  # Limit to 2 meters
            print("Invalid depth value at pixel")
            return None
        print("Depth Intrinsics", depth_intrinsics)
        point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [u, v], depth)
        print("Point", point)
        return np.array(point)

    except Exception as e:
        print(f"Error getting 3D coordinates: {e}")
        return None


def test_getHandPose(robot_pose):
    """Extract translation in mm and rotation angles in degrees."""
    x, y, z = robot_pose[:3, 3] * 1000  # Convert to mm
    R_matrix = robot_pose[:3, :3]
    a, b, c = tr2eul(R_matrix, flip=True, unit="deg")
    return x, y, z, a, b, c


def main():
    # Initialize systems
    pipeline, align, profile = initialize_realsense()
    # yolo_model = YOLO("/home/saketh/camera_cal/src/SimpleHandEye/prod/candetection.pt")
    yolo_model = YOLO("yolov8l-world.pt")
    yolo_model.set_classes(["bottle"])  # Only detect cup
    yolo_model.to("cuda" if torch.cuda.is_available() else "cpu")
    # yolo_model.set_classes(["cup"])  # Only detect cups
    # robot = CobotClient()
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
            depth_intrinsics = (
                depth_frame.profile.as_video_stream_profile().get_intrinsics()
            )

            # YOLO detection for cups only
            results = yolo_model(color_image, conf=0.4)[0]

            if len(results.boxes) > 0:
                # Get first cup detection
                box = results.boxes[0]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = float(box.conf[0])

                # Calculate center
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                # Get 3D position from center
                translation = get_xyz_from_pixel(
                    depth_frame, (center_x, center_y), depth_intrinsics
                )

                if translation is not None:
                    # Update B matrix with new translation
                    B = np.eye(4)
                    B_current = B.copy()
                    B_current[:3, 3] = translation
                    B_matrices.append(B_current)
                    sample_count += 1
                    print(f"Collected sample {sample_count}/{num_samples}")
                    print(
                        f"Translation: X={translation[0]:.3f}, Y={translation[1]:.3f}, Z={translation[2]:.3f}"
                    )

                    # Draw detection
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(color_image, (center_x, center_y), 5, (255, 0, 0), -1)
                    cv2.putText(
                        color_image,
                        f"Can: {confidence:.2f}",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        color_image,
                        f"Samples: {sample_count}/{num_samples}",
                        (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2,
                    )

            # Display the frame
            cv2.imshow("Cup Detection", color_image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        # Process collected samples
        print("\nProcessing collected samples...")
        avg_B = average_poses(B_matrices)

        # Calculate final robot pose
        A = Y @ avg_B @ np.linalg.inv(X)

        x, y, z, a, b, c = test_getHandPose(A)

        print("\nSending averaged pose to robot:")
        print(f"X={x:.2f}mm, Y={y:.2f}mm, Z={z:.2f}mm")
        print(f"A={a:.2f}°, B={b:.2f}°, C={c:.2f}°")
        print("\n")
        grasp = input("Enter grasp type-horizontal or vertical (h/v): ").strip().lower()
        if grasp == "h":
            a, b, c = 90, -179, 0
            print(
                f"GRASP LOCATION: X={x:.2f}mm, Y={y:.2f}mm, Z={z+100:.2f}mm, A={a:.2f}°, B={b:.2f}°, C={c:.2f}°"
            )
        elif grasp == "v":
            a, b, c = 0, -179, 0
            print(
                f"GRASP LOCATION: X={x:.2f}mm, Y={y+100:.2f}mm, Z={z:.2f}mm, A={a:.2f}°, B={b:.2f}°, C={c:.2f}°"
            )
        else:
            print(
                "Invalid grasp type entered. Please enter 'h' for horizontal or 'v' for vertical."
            )

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

#