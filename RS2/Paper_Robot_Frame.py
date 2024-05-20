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

# Vị trí của camera so với robot (thay thế bằng giá trị thực tế - đơn vị: mm) 
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
    
    _MARKER_LENGTH = 0.029  # Chiều dài cạnh của marker ArUco (mét)
    _OFFSET = 0.05  # Khoảng cách từ marker đến tâm khu vực (mét)

    def __init__(self, camera_pose=None) -> None:
        rospy.init_node('camera_display', anonymous=True)  # Khởi tạo node ROS
        self.bridge = CvBridge()  # Khởi tạo CvBridge để chuyển đổi ảnh ROS sang OpenCV

        # Subscribers
        self.rgb_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",  # Topic chứa ảnh màu từ camera
            Image,
            self.img_callback)  # Hàm xử lý khi nhận được ảnh mới

        # Lấy thông tin camera
        self.camera_info = rospy.wait_for_message(
            "/camera/color/camera_info",  # Topic chứa thông tin camera
            CameraInfo)
        self._dist_coeffs = np.array(self.camera_info.D)  # Hệ số méo của camera
        self._camera_matrix = np.array(self.camera_info.K).reshape(3, 3)  # Ma trận nội tham số camera

        # Khởi tạo tf broadcaster để phát thông tin biến đổi tọa độ
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Khởi tạo bộ phát hiện ArUco marker
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)  # Sử dụng dictionary 4x4_250
        self.parameters = aruco.DetectorParameters()  # Tham số phát hiện marker

        # Tạo buffer point để tính toán tư thế
        self.rvec = np.zeros((3, 1))  # Vector xoay
        self.tvec = np.zeros((3, 1))  # Vector tịnh tiến
        self.pose_buffer = np.array([
            [-self._MARKER_LENGTH / 2, self._MARKER_LENGTH / 2, 0],  # Góc dưới bên trái
            [self._MARKER_LENGTH / 2, self._MARKER_LENGTH / 2, 0],  # Góc dưới bên phải 
            [self._MARKER_LENGTH / 2, -self._MARKER_LENGTH / 2, 0],  # Góc trên bên phải
            [-self._MARKER_LENGTH / 2, -self._MARKER_LENGTH / 2, 0]],  # Góc trên bên trái
            dtype=np.float64)  # Tọa độ 4 góc của khu vực đánh dấu


    def clean_shutdown():
        """
        Hàm dọn dẹp khi node tắt.
        """
        print("Shutting down camera_display node.")
        cv2.destroyAllWindows()

    def img_callback(self, data):
        """
        Hàm xử lý khi nhận được ảnh mới từ camera.
        """
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Chuyển đổi ảnh ROS sang OpenCV

            # Phát hiện ArUco marker trong ảnh
            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters)

            # Kiểm tra nếu phát hiện đủ 4 marker
            if ids is not None and len(ids) == 4:
                # Vẽ marker lên ảnh
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

                # Tính toán tâm của khu vực được đánh dấu bởi 4 marker
                # center = np.mean([np.mean(corner, axis=0) for corner in corners], axis=0)
                center = corners[np.where(ids == 4)[0][0]]
                paper_center = np.array([np.mean(center[:, 0]), np.mean(center[:, 1])])  # [u, v]
                rospy.loginfo(paper_center)  # In ra tọa độ 2D của tâm 
                _, self.rvec, self.tvec = cv2.solvePnP(self.pose_buffer, center, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec)
                cv2.drawFrameAxes(cv_image, self._camera_matrix, self._dist_coeffs, self.rvec, self.tvec, 0.1)

                # Độ sâu thực tế từ camera đến tờ giấy (đơn vị: mm)
                depth_mm = 0.5 * 1000  

                # Vị trí của camera so với robot (thay thế bằng giá trị thực tế - đơn vị: mm) 
                camera_pose = (32.5, 107, 25)

                # Tính toán vị trí 3D của tờ giấy so với robot  
                paper_pose_wrt_robot = get_paper_pose_wrt_robot(
                    paper_center[0], paper_center[1], depth_mm, camera_pose, 
                    self._camera_matrix, self._dist_coeffs
                )
                rospy.loginfo(f"Vị trí tờ giấy (robot): {paper_pose_wrt_robot}")    


        except CvBridgeError as err:
            rospy.logerr(err)
            return

        # Hiển thị ảnh (debug)
        cv2.imshow('Test_head_Cam', cv_image)
        cv2.waitKey(3)


def img_to_cam_3d(x, y, depth, camera_matrix, dist_coeffs):
    """
    Chuyển đổi điểm 2D trên ảnh sang điểm 3D trong hệ quy chiếu của camera.

    Args:
        x (float): Tọa độ x của điểm trên ảnh.
        y (float): Tọa độ y của điểm trên ảnh.
        depth (float): Giá trị độ sâu tại điểm (x, y) (mm).
        camera_matrix (numpy.ndarray): Ma trận nội tham số của camera (3x3).
        dist_coeffs (numpy.ndarray): Hệ số méo của camera.

    Returns:
        numpy.ndarray: Điểm 3D trong hệ quy chiếu của camera (x, y, z) (mm).
    """
    # 1. Khử méo điểm ảnh
    undistorted_point = cv2.undistortPoints(np.array([[x, y]]), camera_matrix, dist_coeffs)[0, 0]

    # 2. Áp dụng mô hình camera lỗ kim
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # Độ dài tiêu cự
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # Điểm chính

    x_normalized = (undistorted_point[0] - cx) / fx
    y_normalized = (undistorted_point[1] - cy) / fy

    # 3. Tính toán tọa độ 3D
    x_3d = x_normalized * depth
    y_3d = y_normalized * depth
    z_3d = depth

    return np.array([x_3d, y_3d, z_3d])  


def get_paper_pose_wrt_robot(u, v, depth, camera_pose, camera_matrix, dist_coeffs):
    """
    Tính toán vị trí 3D của tờ giấy so với robot.

    Args:
        u, v: Tọa độ 2D của tâm tờ giấy trên ảnh.
        depth: Giá trị độ sâu tại điểm (u, v) (mm).
        camera_pose: Vị trí 3D của camera so với robot (x, y, z) (mm).
        camera_matrix: Ma trận nội tham số của camera.
        dist_coeffs: Hệ số méo của camera.

    Returns:
        Mảng NumPy: Vị trí 3D của tờ giấy so với robot (x, y, z) (mm).
    """
    # Chuyển đổi tọa độ 2D và độ sâu thành tọa độ 3D trong hệ quy chiếu camera
    point_3d_camera = img_to_cam_3d(u, v, depth, camera_matrix, dist_coeffs)

    # Chuyển đổi tọa độ 3D từ hệ quy chiếu camera sang hệ quy chiếu robot
    # (Giả sử camera_pose là vector tịnh tiến, cần bổ sung ma trận xoay nếu cần)
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