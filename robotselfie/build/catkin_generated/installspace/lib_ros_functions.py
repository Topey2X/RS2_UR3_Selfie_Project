#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from robotselfie.msg import ContourList, Contour
from typing import list, tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ROSNODE_img_processor:
    def __init__(self, process_image_func):
        rospy.init_node('img_processor', anonymous=True)
        self.process_image = process_image_func
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.contour_pub = rospy.Publisher('contours', ContourList, queue_size=10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        contours = self.process_image(img)
        self.publish_contours(contours)

    def publish_contours(self, contours):
        contour_list_msg = ContourList()
        for contour in contours:
            contour_msg = Contour()
            contour_msg.points = [Point(x=point[0], y=point[1], z=0.0) for point in contour]
            contour_list_msg.contours.append(contour_msg)
        self.contour_pub.publish(contour_list_msg)

    def run(self):
        rospy.spin()

# ... (rest of the lib_ros_functions.py code)