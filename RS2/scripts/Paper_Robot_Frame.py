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

# Position of the camera relative to the robot - EOF (replace with actual value - unit: mm)
camera_to_robot = np.identity(4) @ np.array((32.5, 107, 25, 1))   

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

class ArUcoMarker():
    
    _MARKER_LENGTH = 0.029  # Length of the ArUco marker's side (meters)
    _OFFSET = 0.05  # Distance from the marker to the center of the area (meters)
    
    def __init__(self, camera_pose=None) -> None:
        rospy.init_node('camera_display', anonymous=True)  # Initialize ROS node
        self.bridge = CvBridge()  # Initialize CvBridge to convert ROS images to OpenCV


        # Subscribers
        self.rgb_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",  # Topic containing color images from the camera
            Image,
            self.img_callback)  # Callback function when a new image is received


        # Get camera info
        self.camera_info = rospy.wait_for_message(
            "/camera/color/camera_info",  # Topic containing camera info
            CameraInfo)
        self._dist_coeffs = np.array(self.camera_info.D)  # Camera distortion coefficients
        self._camera_matrix = np.array(self.camera_info.K).reshape(3, 3)  # Camera intrinsic matrix
        
        # Initialize tf broadcaster to broadcast coordinate transformations
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Initialize ArUco marker detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)  # Use dictionary 4x4_250
        self.parameters = aruco.DetectorParameters()  # Marker detection parameters

         # Create pose buffer for pose calculation
        self.rvec = np.zeros((3, 1))  # Rotation vector
        self.tvec = np.zeros((3, 1))  # Translation vector
        self.pose_buffer = np.array([
            [-self._MARKER_LENGTH / 2, self._MARKER_LENGTH / 2, 0],  # Bottom-left corner
            [self._MARKER_LENGTH / 2, self._MARKER_LENGTH / 2, 0],   # Bottom-right corner
            [self._MARKER_LENGTH / 2, -self._MARKER_LENGTH / 2, 0],  # Top-right corner
            [-self._MARKER_LENGTH / 2, -self._MARKER_LENGTH / 2, 0]],  # Top-left corner
            dtype=np.float64)  # Coordinates of the 4 corners of the marked area


    def clean_shutdown():
        """
        Cleanup function when the node shuts down.
        """
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    def img_callback(self, data):
        """
        Callback function when a new image is received from the camera.
        """

        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Convert ROS image to OpenCV

            # Detect ArUco markers in the image
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters)

            # Check if 4 markers are detected
            if ids is not None and len(ids) == 4:
                # Draw markers on the image
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                # Calculate the center of the area marked by the 4 markers
                # center = np.mean([np.mean(corner, axis=0) for corner in corners], axis=0)
                center = corners[np.where(ids == 4)[0][0]]
                paper_center = np.array([np.mean(center[:, 0]), np.mean(center[:, 1])])  # [u, v]
                rospy.loginfo(paper_center)  # Print 2D coordinates of the center 
                _, self.rvec, self.tvec = cv2.solvePnP(self.pose_buffer, center, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec)
                cv2.drawFrameAxes(cv_image, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec, 0.1)

                # Actual depth from camera to the paper (unit: mm)
                depth_mm = 0.5 * 1000  

                # Position of the camera relative to the robot (replace with actual value - unit: mm) 
                camera_pose = (32.5, 107, 25)

                # Calculate the 3D position of the paper relative to the robot  
                paper_pose_wrt_robot = get_paper_pose_wrt_robot(
                    paper_center[0], paper_center[1], depth_mm, camera_pose, 
                    self._camera_matrix, self._dist_coeffs
                )
                rospy.loginfo(f"Vị trí tờ giấy (robot): {paper_pose_wrt_robot}")    


        except CvBridgeError as err:
            rospy.logerr(err)
            return

        # Display the image (debug)
        cv2.imshow('Test_head_Cam', cv_image)
        cv2.waitKey(3)


def img_to_cam_3d(x, y, depth, camera_matrix, dist_coeffs):
    """
    Converts a 2D point on the image to a 3D point in the camera frame.

    Args:
        x (float): x coordinate of the point on the image.
        y (float): y coordinate of the point on the image.
        depth (float): Depth value at the point (x, y) (mm).
        camera_matrix (numpy.ndarray): Camera intrinsic matrix (3x3).
        dist_coeffs (numpy.ndarray): Camera distortion coefficients.

    Returns:
        numpy.ndarray: 3D point in the camera frame (x, y, z) (mm).
    """
    # 1. Undistort the image point
    undistorted_point = cv2.undistortPoints(np.array([[x, y]]), camera_matrix, dist_coeffs)[0, 0]

    # 2. Apply the pinhole camera model
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # Focal lengths
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # Principal point
    x_normalized = (undistorted_point[0] - cx) / fx
    y_normalized = (undistorted_point[1] - cy) / fy

     # 3. Calculate the 3D coordinates
    x_3d = x_normalized * depth
    y_3d = y_normalized * depth
    z_3d = depth

    return np.array([x_3d, y_3d, z_3d])  


def get_paper_pose_wrt_robot(u, v, depth, camera_pose, camera_matrix, dist_coeffs):
    """
    Calculates the 3D position of the paper relative to the robot.

    Args:
        u, v: 2D coordinates of the center of the paper on the image.
        depth: Depth value at the point (u, v) (mm).
        camera_pose: 3D position of the camera relative to the robot (x, y, z) (mm).
        camera_matrix: Camera intrinsic matrix.
        dist_coeffs: Camera distortion coefficients.

    Returns:
        numpy.ndarray: 3D position of the paper relative to the robot (x, y, z) (mm).
    """
    # Convert 2D coordinates and depth to 3D coordinates in the camera frame
    point_3d_camera = img_to_cam_3d(u, v, depth, camera_matrix, dist_coeffs)

    # Convert 3D coordinates from the camera frame to the robot frame
    # (Assuming camera_pose is a translation vector, add a rotation matrix if needed)
    point_3d_robot = point_3d_camera + np.array(camera_pose)

    return point_3d_robot

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


def main():    
    ArUcoMarker()
    rospy.on_shutdown(ArUcoMarker.clean_shutdown)
    
    paper_to_camera = np.array([[1, 0, 0, 0.5],
                                [0, 1, 0, 0.2],
                                [0, 0, 1, 0.7],
                                [0, 0, 0, 1]])

    ######## lay transform de chuyen tu paper sang he toa do cua robot
    # camera_to_robot : vi tri cua camera so voi robot
    # paper_to_camera: vi tri cua paper so voi camera --> co tu doc aruco
    
    # paper_to_base_transform = get_paper_to_base_transform(camera_to_robot, paper_to_camera)
    
    # ######### Get the final points to send to robot for drawing
    # # csv_file: tap hop cac diem anh mm trong toa do cua to giay
    # # transform_matrix: ma tran de chuyen tu` vi tri diem trong paper sang vi tri diem so voi robot
    # final_points = apply_transform_csv(csv_file = 'Waypoints_mm.csv', transform_matrix = paper_to_base_transform)
    
    # print(final_points)
        
    rospy.spin()
    
    

if __name__ == '__main__':
    main()