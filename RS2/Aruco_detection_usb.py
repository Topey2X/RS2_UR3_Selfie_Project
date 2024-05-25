import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

import tf
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped


class ArUcoMarker():

    _MARKER_LENGTH = 0.029  # Marker length in meters
    _OFFSET = 0.05  # Offset from marker to center of boundary

    def __init__(self, camera_pose = None) -> None:

        rospy.init_node('camera_display', anonymous=True)
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_subscriber = rospy.Subscriber(
            "/camera/color/image_raw", 
            Image, 
            self.img_callback)
        
        # self.depth_callback
        
        self.camera_info = rospy.wait_for_message(
            "/camera/color/camera_info", 
            CameraInfo)
        
        self._dist_coeffs = np.array(self.camera_info.D)
        self._camera_matrix = np.array(self.camera_info.K).reshape(3, 3)


        # Setup tf2 broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()


        # Setup aruco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        
        # Setup buffer point for pose
        self.rvec = np.zeros((3, 1))
        self.tvec = np.zeros((3, 1))
        self.pose_buffer = np.array([[-self._MARKER_LENGTH/2, self._MARKER_LENGTH/2, 0],    # Bottom-left corner
                                     [self._MARKER_LENGTH/2, self._MARKER_LENGTH/2, 0],      # Bottom-right corner
                                     [self._MARKER_LENGTH/2, -self._MARKER_LENGTH/2, 0],    # Top-right corner
                                     [-self._MARKER_LENGTH/2, -self._MARKER_LENGTH/2, 0]],       # Top-left corner
                                    dtype=np.float64)
        

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    def img_callback(self, data):
            """
            Callback function for the pose of the area marker.
            """
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                corners, ids, _ = cv2.aruco.detectMarkers(
                    cv_image, self.aruco_dict, parameters=self.parameters)
                

                # Check if four markers are detected
                # if ids is not None and 1 <= len(ids) <= 4:
                if ids is not None and len(ids) == 4:

                    # Draw markers
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)


                    # Calculate center of boundary formed by markers
                    # center = np.mean([np.mean(corner, axis=0)
                    #                 for corner in corners], axis=0)
                    # print(np.where(ids == 4)[0][0])
                    center = corners[np.where(ids == 4)[0][0]]
                    # center = corners[0]

                    # print("CONNER:")
                    # print(corners)
                    
                    # rospy.loginfo(center)
                    
                    paper_center = np.array([np.mean(center[:,0]), np.mean(center[:,1])])
                    rospy.loginfo(paper_center)
                    
                    # point3d = func(paper_center[0], paper_center[1], depth)
                    
                    # Calculate pose of paper center related to camera frame
                    _, self.rvec, self.tvec = cv2.solvePnP(self.pose_buffer, center, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec)
                    cv2.drawFrameAxes(cv_image, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec, 0.1)

                    
                    # Set orientation (if you have orientation data)
                    rotation_matrix, _ = cv2.Rodrigues(self.rvec)
                    rot_mat_4x4 = tf.transformations.identity_matrix()
                    rot_mat_4x4[:3, :3] = rotation_matrix
                    ori_in_quat = tf.transformations.quaternion_from_matrix(rot_mat_4x4)


                    
                    # Broadcast transform
                    self.tf_broadcaster.sendTransform((self.tvec[0], self.tvec[1], self.tvec[2]),
                                                    ori_in_quat,
                                                    rospy.Time.now(),
                                                    "aruco_marker",
                                                    "camera_color_optical_frame")
                    
                    rospy.loginfo(f"x = {self.tvec[0]},y =  {self.tvec[1]},z =  {self.tvec[2]}")

                    

            except CvBridgeError as err:
                rospy.logerr(err)
                return

            # Imshow for debugging purposes
            cv2.imshow('Test_head_Cam', cv_image)
            cv2.waitKey(3)


def main():

    ArUcoMarker()
    rospy.on_shutdown(ArUcoMarker.clean_shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()



# # Create function from 2D to 3D camera
# # Typical values for RealSense D435 camera
# RGB_RESOLUTION = (1920, 1080)  # Width, Height
# DEPTH_RESOLUTION = (1280, 720)
# RGB_FOCAL_LENGTH = (615, 615)  # Approximate horizontal, vertical focal lengths (in pixels)
# DEPTH_FOCAL_LENGTH = (617, 617)  # Approximate horizontal, vertical focal lengths (in pixels)

# # Camera intrinsic matrices (assuming principal point at the center of the image)
# RGB_CAMERA_MATRIX = np.array([[RGB_FOCAL_LENGTH[0], 0, RGB_RESOLUTION[0] / 2],
#                               [0, RGB_FOCAL_LENGTH[1], RGB_RESOLUTION[1] / 2],
#                               [0, 0, 1]], dtype=np.float32)

# DEPTH_CAMERA_MATRIX = np.array([[DEPTH_FOCAL_LENGTH[0], 0, DEPTH_RESOLUTION[0] / 2],
#                                 [0, DEPTH_FOCAL_LENGTH[1], DEPTH_RESOLUTION[1] / 2],
#                                 [0, 0, 1]], dtype=np.float32)

# # Distortion coefficients (assuming no distortion for simplicity)
# DIST_COEFFS = np.zeros((5, 1))

# def img_to_cam_3d(x, y, depth, camera_matrix, dist_coeffs):
#     """
#     Transforms a 2D image point to a 3D point in the camera coordinate system.

#     Args:
#         x (float): x-coordinate of the image point
#         y (float): y-coordinate of the image point
#         depth (float): depth value corresponding to the image point
#         camera_matrix (numpy.ndarray): 3x3 camera intrinsic matrix
#         dist_coeffs (numpy.ndarray): vector of distortion coefficients

#     Returns:
#         numpy.ndarray: 3D point in the camera coordinate system
#     """
#     # Undistort the 2D image point
#     undist_point = cv2.undistortPoints(np.array([[x, y]]), camera_matrix, dist_coeffs)[0, 0]

#     # Apply the pinhole camera model to get the 3D ray
#     fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
#     cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    
#     x_norm = (undist_point[0] - cx) / fx
#     y_norm = (undist_point[1] - cy) / fy
    
#     ray_x = x_norm * depth
#     ray_y = y_norm * depth
#     ray_z = depth

#     # Return the 3D point in camera coordinates
#     return np.array([ray_x, ray_y, ray_z])

# # Example usage for RGB camera
# rgb_x, rgb_y = 960, 540  # Example 2D image point in RGB image
# rgb_depth = 1.5  # Example depth value (in meters)
# rgb_3d_point = img_to_cam_3d(rgb_x, rgb_y, rgb_depth, RGB_CAMERA_MATRIX, DIST_COEFFS)
# print(f"RGB 3D Point: {rgb_3d_point}")

# # Example usage for Depth camera
# depth_x, depth_y = 640, 360  # Example 2D image point in Depth image
# depth_value = 1.2  # Example depth value (in meters)
# depth_3d_point = img_to_cam_3d(depth_x, depth_y, depth_value, DEPTH_CAMERA_MATRIX, DIST_COEFFS)
# print(f"Depth 3D Point: {depth_3d_point}")

# #Transform 2D point sang 3D point
