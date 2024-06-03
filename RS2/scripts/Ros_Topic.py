#!/usr/bin/env python

import rospy
from std_msgs.msg import String  # Import appropriate message type

class MyRobotControlNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('my_robot_control_node', anonymous=True)
        
        # Create a subscriber for the /optimized_path topic
        self.path_subscriber = rospy.Subscriber('/optimized_path', String, self.path_callback)
        
        # Create a publisher for the My_Robot_Control topic
        self.control_publisher = rospy.Publisher('My_Robot_Control', String, queue_size=10)
        
        rospy.loginfo("My Robot Control Node has been started.")
        
    def path_callback(self, msg):
        rospy.loginfo(f"Received path: {msg.data}")
        
        # Process the data (for example, we just forward it in this case)
        control_msg = String()
        control_msg.data = f"Control message based on path: {msg.data}"
        
        # Publish the processed data
        self.control_publisher.publish(control_msg)
        rospy.loginfo(f"Published control message: {control_msg.data}")
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MyRobotControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass