"""
Code for manual testing , and it will when aruco is attached to the robot end effector(for differences)

 A = Hand w.r.t Base
 B = Tag w.r.t Camera
 X = Tag w.r.t Hand
 Y = Camera w.r.t Base

"""
import numpy as np
from spatialmath.base import tr2eul
from scipy.spatial.transform import Rotation as R
from CobotClient import CobotClient



# Calibration matrices

# A = Hand w.r.t Base
# B = Tag w.r.t Camera
# X = Tag w.r.t Hand
# Y = Camera w.r.t Base

# Matrix X
X = np.array([
    [ 0.979,  0.035,  0.202, -0.132],
    [-0.035,  0.999, -0.002,  0.002],
    [-0.202, -0.005,  0.979,  0.099],
    [ 0.0,  0.0,  0.0,  1.0]
])

Y = np.array([
    [-0.467,  0.598, -0.652,  0.56],
    [ 0.88,  0.388, -0.274, -0.251],
    [ 0.089, -0.701, -0.707,  0.827],
    [ 0.0,  0.0,  0.0,  1.0]
])

# Matrix B (Tag2Camera)
B = np.array([
    [ 0.928,  0.344, -0.146, -0.153],
    [-0.213,  0.808,  0.549,  0.106],
    [ 0.307, -0.478,  0.823,  0.5  ],
    [ 0.0,    0.0,    0.0,    1.0  ]
])


A = Y @ B @ np.linalg.inv(X)

print("A Matrix:")
print(A)
#%%
def test_getHandPose(robot_pose):
    # Extract translation (convert to mm)
    x = robot_pose[0, 3] * 1000
    y = robot_pose[1, 3] * 1000
    z = robot_pose[2, 3] * 1000

    # Extract the rotation matrix
    R_matrix = robot_pose[:3, :3]
    a,b,c = tr2eul(R_matrix, flip=True, unit='deg')
    return x,y,z,a,b,c
# Extract predicted pose from A using the new test_getHandPose function
x, y, z, a, b, c = test_getHandPose(A)

print("A Matrix:")
print(A)

print(f"\nPredicted Robot Pose (A):")
print(f"X: {x:.2f} mm")
print(f"Y: {y:.2f} mm")
print(f"Z: {z:.2f} mm")
print(f"A: {a:.2f} degrees")
print(f"B: {b:.2f} degrees")
print(f"C: {c:.2f} degrees")

# Comparison with actual robot pose
robot = CobotClient()
current_pose = robot.get_current_pose()

actual_pose = {
    'X': current_pose['X'],
    'Y': current_pose['Y'],
    'Z': current_pose['Z'],
    'A': current_pose['A'],
    'B': current_pose['B'],
    'C': current_pose['C']
}

print("\nActual Robot Pose:")
print(actual_pose)

# Calculate differences between predicted and actual poses
diff = {
    'X': x - actual_pose['X'],
    'Y': y - actual_pose['Y'],
    'Z': z - actual_pose['Z'],
    'A': a - actual_pose['A'],
    'B': b - actual_pose['B'],
    'C': c - actual_pose['C']
}

print("\nDifferences:")
for key, value in diff.items():
    print(f"{key}: {value:.2f}")


# Differences:
# X: -44.75
# Y: -6.15
# Z: -18.86
# A: -0.80
# B: -1.67
# C: -0.64
# X: -34.48
# Y: 0.17
# Z: -22.62
# A: 0.88
# B: -0.02
# C: 0.62
## adding offset of 30 mm
# A Matrix:
# [[-0.4694831   0.88342519  0.01744951 -0.00241329]
#  [ 0.88320019  0.46929466 -0.007675   -0.38767253]
#  [-0.01507649  0.01073323 -0.99904578  0.44411855]
#  [ 0.          0.          0.          1.        ]]