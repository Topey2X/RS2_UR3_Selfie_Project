import numpy as np
import pandas as pd
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import tf
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped
import csv

def apply_transform_csv(csv_file, transform_matrix):
    """
    Reads a CSV file containing x, y, z coordinates, applies a 4x4 transformation matrix,
    and returns the transformed coordinates as a NumPy array.

    Args:
        csv_file (str): Path to the CSV file containing x, y, z coordinates.
        transform_matrix (numpy.ndarray): 4x4 NumPy array representing the transformation matrix.

    Returns:
        numpy.ndarray: NumPy array containing the transformed x, y, z coordinates.
    """
    # Read the CSV file
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip the header
        data = list(reader)

    # Convert the data to a NumPy array
    coords = np.array(data, dtype=float)

    # Add a column of ones to the coordinates for homogeneous coordinates
    coords = np.column_stack((coords, np.ones(len(coords))))

    # Apply the transformation matrix
    transformed_coords = np.dot(transform_matrix, coords.T).T

    # Extract the x, y, z coordinates from the transformed homogeneous coordinates
    transformed_xyz = transformed_coords[:, :3]

    return transformed_xyz
        
def get_paper_to_base_transform(camera_to_base, paper_to_camera):
    """
    Calculates the transform of the paper with respect to the robot base.

    Args:
        camera_to_base (numpy.ndarray): 4x4 NumPy array representing the camera pose with respect to the robot base.
        paper_to_camera (numpy.ndarray): 4x4 NumPy array representing the paper pose with respect to the camera.

    Returns:
        numpy.ndarray: 4x4 NumPy array representing the transform of the paper with respect to the robot base.
    """
    paper_to_base = np.dot(camera_to_base, paper_to_camera)
    return paper_to_base

# Measure again on a real Robot
camera_to_robot = np.array([[1, 0, 0, 0.405],
                            [0, 1, 0, 0.28],
                            [0, 0, 1, 0.1],
                            [0, 0, 0, 1]])

# Get it from Aruco
paper_to_camera = np.array([[1, 0, 0, 0.5],
                            [0, 1, 0, 0.2],
                            [0, 0, 1, 0.7],
                            [0, 0, 0, 1]])

paper_to_base_transform = get_paper_to_base_transform(camera_to_robot, paper_to_camera)

######### Get the final points to send to robot for drawing
# csv_file: Set of mm pixels in the coordinates of the paper
# transform_matrix: Matrix to convert from point position in paper to point position relative to robot

final_points = apply_transform_csv(csv_file = 'Waypoints_mm_scaled.csv', transform_matrix = paper_to_base_transform)
print(final_points)

# Save the final_points array to a new CSV file
np.savetxt('Final_Waypoints_Robot.csv', final_points, fmt='%.4f', delimiter=',', header='x,y,z', comments='')

# Print the values with the specified format
with open('Final_Waypoints_Robot.csv', 'r') as file:
    lines = file.readlines()

for i, line in enumerate(lines):
    values = line.strip().split(',')
    x, y, z = values
    print(f"The line {i+1} has x = {x}, y = {y}, and z = {z}")

# Find and print the minimum and maximum values ​​of x and y
x_values = final_points[:, 0]
y_values = final_points[:, 1]

x_min, x_max = x_values.min(), x_values.max()
y_min, y_max = y_values.min(), y_values.max()

print(f"Value of x from {x_min:.4f} to {x_max:.4f}")
print(f"Value of y from {y_min:.4f} to {y_max:.4f}")