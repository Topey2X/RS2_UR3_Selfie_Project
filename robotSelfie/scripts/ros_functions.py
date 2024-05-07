#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point32
from robotselfie.msg import ContourList, Contour # TODO
from typing import list, tuple

def publish_contours(contours : list[list[tuple[float, float]]]):
    rospy.init_node('contour_publisher', anonymous=True)
    pub = rospy.Publisher('contours', ContourList, queue_size=10)

    # Build the message
    contour_list_msg = ContourList()
    for contour in contours:
        contour_msg = Contour()
        contour_msg.contour = [Point32(x, y, 0.0) for x, y in contour]
        contour_list_msg.contours.append(contour_msg)

    # Publish the message
    pub.publish(contour_list_msg)
    rospy.loginfo("Contours published, node will now shutdown.")
    rospy.signal_shutdown("Completed single message publication")
    
if __name__ == '__main__':
  contours = [
        [(1.0, 2.0), (3.0, 4.0)],
        [(5.0, 6.0), (7.0, 8.0)]
    ]
  try:
      publish_contours(contours)
  except rospy.ROSInterruptException:
      pass