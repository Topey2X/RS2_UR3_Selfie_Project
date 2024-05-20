#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import numpy as np
import tf.transformations

import csv
from collections import namedtuple
import pandas as pd

class UR3eController:
    def __init__(self):
        # Định nghĩa định hướng ban đầu
        self.start_orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)

    def read_csv(self):
        # Define a namedtuple to represent a Point
        Point = namedtuple('Point', ['x', 'y', 'z'])

        # Read the CSV file into a DataFrame
        df = pd.read_csv('Final_Waypoints_Robot.csv', engine='python')

        # Convert the DataFrame to a list of Point namedtuples
        points = [Point(*row) for row in df.values]

        return points

    def move_ur3e_by_pose(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3e_node', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.2

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        rospy.sleep(5)

    def move_ur3e_by_joints(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3e_by_joints_node', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -90*3.14/180
        joint_goal[2] = 56*3.14/180
        joint_goal[3] = -94*3.14/180
        joint_goal[4] = -98*3.14/180
        joint_goal[5] = 25*3.14/180

        move_group.go(joint_goal, wait=True)
        move_group.stop()

        rospy.sleep(5)

    def move_ur3e_cartesian_path(self, waypoints, fraction=1.0, maxVelocity=0.05):
        """
        Move the UR3e robot arm along a specified Cartesian path.

        Parameters:
        - waypoints: A list of geometry_msgs/Pose objects defining the path points.
        - fraction: Desired fraction of the path to execute (default is 1.0, full path).
        - maxVelocity: Maximum velocity scaling factor for the arm movement.
        """
        # Initialize moveit_commander if not already done
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the node, planning scene, and robot commander
        rospy.init_node('ur3e_cartesian_path_node', anonymous=True)
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("manipulator")

        # Set the robot reference frame and end effector
        arm.set_pose_reference_frame("base_link")
        arm.set_end_effector_link("flange")

        # Set the maximum velocity scaling factor for the robot arm
        arm.set_max_velocity_scaling_factor(maxVelocity)

        # Plan the Cartesian path connecting the waypoints
        (plan, fraction) = arm.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.01,        # eef_step, resolution of 1 cm
                                 0.0,         # jump_threshold, disabled
                                 False)       # avoid_collisions

        # Execute the plan to move along the computed Cartesian path
        if fraction >= 0.9:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only {}% achieved".format(fraction * 100))

        return fraction

if __name__ == '__main__':
    try:
        # Khởi tạo đối tượng UR3eController
        ur3e_controller = UR3eController()

        ## Control the robot by pose
        # ur3e_controller.move_ur3e_by_pose()

        ## Control the robot by joint state
        # ur3e_controller.move_ur3e_by_joints()

        # Control the robot by way points
        # Define the example way points

        # Test >1000 point
        waypoints = []
        point_list = ur3e_controller.read_csv()

        count = 0

        # Chuyển đổi từ Quaternion sang ma trận xoay
        rot_matrix = tf.transformations.quaternion_matrix([ur3e_controller.start_orientation.x,
                                                           ur3e_controller.start_orientation.y,
                                                           ur3e_controller.start_orientation.z,
                                                           ur3e_controller.start_orientation.w])

        for i, point in enumerate(point_list):
            if count % 2 == 0:
                pose = Pose()
                pose.position.x = round(point.x, 3) / 1000
                pose.position.y = round(point.y, 3) / 1000
                pose.position.z = round(point.z, 3) / 1000

                # Thiết lập định hướng cho waypoint
                pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_matrix(rot_matrix))

                waypoints.append(pose)

            if count == 20:
                break
            count += 1

        for pose in waypoints:
            print("Pose:", pose)

        # Gọi phương thức move_ur3e_cartesian_path trên đối tượng ur3e_controller
        ur3e_controller.move_ur3e_cartesian_path(waypoints)

        # Test private
        # ur3e_controller.move_ur3e_by_joints()

    except rospy.ROSInterruptException:
        pass