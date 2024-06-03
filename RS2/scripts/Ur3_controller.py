#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import numpy as np
import csv
import pandas as pd
# def read_csv(nearestLineMM):
#     """Reads point data from a CSV file and converts units from mm to m"""
#     data = []
#     with open(nearestLineMM, 'r') as csvfile:
#         # Read the entire file content as a string
#         content = csvfile.read()
#         # Split the string by commas and newlines
#         rows = [line.split(',') for line in content.splitlines()]
#         for row in rows:
#             # Filter out empty values before converting to floats
#             filtered_row = [v for v in row if v]
#             if len(filtered_row) == 3:
#                 x, y, z = map(lambda v: float(v) / 1000, filtered_row)  # Convert mm to m
#                 data.append([x, y, z])
#     return data
def move_ur3e_by_pose():
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

def move_ur3e_by_joints():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur3e_by_joints_node', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -45*3.14/180
    joint_goal[2] = 56*3.14/180
    joint_goal[3] = -94*3.14/180
    joint_goal[4] = -98*3.14/180
    joint_goal[5] = 25*3.14/180

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    rospy.sleep(5)

def move_ur3e_cartesian_path(waypoints, fraction=1.0, maxVelocity=0.1):
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
                             True)        # avoid_collisions

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
        # move_ur3e_by_pose()
        # move_ur3e_by_joints()
        waypoints = []
        pose = Pose()
        pose.position.x = 0.2
        pose.position.y = 0.3
        pose.position.z = 0.2
        pose.orientation.w = 1.0
        waypoints.append(pose)
        
        pose = Pose()
        pose.position.x = 0.1
        pose.position.y = 0.3
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        waypoints.append(pose)
        
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.1
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        waypoints.append(pose)



        # pose = Pose()
        # pose.position.x = 58.0901/1000
        # pose.position.y = 147.0046/1000
        # pose.position.z = 0.8000/1000
        # pose.orientation.w = 1.0
        # waypoints.append(pose)
        
        # pose = Pose()
        # pose.position.x = 58.6596/1000
        # pose.position.y = 148.3154/1000
        # pose.position.z = 0.8000/1000
        # pose.orientation.w = 1.0
        # waypoints.append(pose)
        
        # pose = Pose()
        # pose.position.x = 59.1915/1000
        # pose.position.y = 149.3076/1000
        # pose.position.z = 0.8000/1000
        # pose.orientation.w = 1.0
        # waypoints.append(pose)
        
        # pose = Pose()
        # pose.position.x =60.8551/1000
        # pose.position.y = 151.7254/1000
        # pose.position.z = 0.8000/1000
        # pose.orientation.w = 1.0
        # waypoints.append(pose)
        
        
        # pose = Pose()
        # pose.position.x = 0.2 
        # pose.position.y = 0.3 
        # pose.position.z = 0.45
        # pose.orientation.w = 1.0
        # waypoints.append(pose)

        # pose = Pose()
        # pose.position.x = 0.3 
        # pose.position.y = 0.25 
        # pose.position.z = 0.2
        # pose.orientation.w = 1.0
        # waypoints.append(pose)

        
        # -5 -39 58 242 270 -115
        # Assuming 'self' is an instance of a class that includes the move_ur3e_cartesian_path method
        move_ur3e_cartesian_path(waypoints)

    except rospy.ROSInterruptException:
        pass
