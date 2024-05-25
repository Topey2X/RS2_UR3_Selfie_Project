#!/usr/bin/env python
import rospy
import moveit_commander
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
import sys
import time

class UR3Controller:
    def __init__(self):
        # Initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3_controller', anonymous=True)

        # Initialize RobotCommander and PlanningSceneInterface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize MoveGroupCommander for the UR3 arm
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame for pose targets
        self.move_group.set_pose_reference_frame("base_link")

        # Set the tolerance for position and orientation
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)

    def read_waypoints(self, csv_file):
        """
        Reads waypoints from a CSV file and returns them as a list of Pose objects.
        """
        df = pd.read_csv(csv_file)

        # Tính giá trị nhỏ nhất và lớn nhất của x và y
        x_min, x_max = df['x'].min(), df['x'].max()
        y_min, y_max = df['y'].min(), df['y'].max()
        rospy.loginfo(f"Giá trị của x từ {x_min:.4f} đến {x_max:.4f}")
        rospy.loginfo(f"Giá trị của y từ {y_min:.4f} đến {y_max:.4f}")

        waypoints = []
        for index, row in df.iterrows():
            pose = Pose()
            pose.position.x = row['x'] / 1000.0  # Convert mm to meters
            pose.position.y = row['y'] / 1000.0  # Convert mm to meters
            pose.position.z = row['z'] / 1000.0  # Convert mm to meters

            # Assuming the orientation remains constant, e.g., pointing down to the paper
            quaternion = quaternion_from_euler(0, np.pi, 0)  # Adjust as needed
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            waypoints.append(pose)
        return waypoints

    def move_to_waypoints(self, waypoints):
        """
        Moves the UR3 arm through the given waypoints using Cartesian path.
        """
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.005,       # eef_step - increased number of steps
            0.0          # jump_threshold
        )
        if fraction < 1.0:
            rospy.logwarn(f"Only {fraction*100:.2f}% of the path was planned successfully.")
        # Execute the plan
        result = self.move_group.execute(plan, wait=True)
        if not result:
            rospy.logerr("Path execution failed.")
        else:
            rospy.loginfo("Path execution succeeded.")

if __name__ == '__main__':
    try:
        ur3_controller = UR3Controller()
        waypoints = ur3_controller.read_waypoints('Final_Waypoints_Robot.csv')
        
        # Log waypoints for debugging
        rospy.loginfo(f"Loaded {len(waypoints)} waypoints from 'Final_Waypoints_Robot.csv'.")
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo(f"Waypoint {i+1}: x={waypoint.position.x:.4f}, y={waypoint.position.y:.4f}, z={waypoint.position.z:.4f}")

        # In ra giá trị nhỏ nhất và lớn nhất của x và y
        df = pd.read_csv('Final_Waypoints_Robot.csv')
        x_min, x_max = df['x'].min(), df['x'].max()
        y_min, y_max = df['y'].min(), df['y'].max()
        rospy.loginfo(f"Giá trị của x từ {x_min:.4f} đến {x_max:.4f}")
        rospy.loginfo(f"Giá trị của y từ {y_min:.4f} đến {y_max:.4f}")
        
        # Dừng 5 giây trước khi tiếp tục
        time.sleep(5)
        
        ur3_controller.move_to_waypoints(waypoints)
        rospy.loginfo("Path execution complete.")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()



#!/usr/bin/env python

import rospy
import moveit_commander
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
import sys
import time

class UR3Controller:
    def __init__(self):
        # Initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3_controller', anonymous=True)

        # Initialize RobotCommander and PlanningSceneInterface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize MoveGroupCommander for the UR3 arm
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame for pose targets
        self.move_group.set_pose_reference_frame("base_link")

        # Set the tolerance for position and orientation
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)
        
    def go_to_joint_angle(self, joint_angle):
        
        self.move_group.set_joint_value_target(joint_angle)
        self.move_group.go()
        pass

    def read_waypoints(self, csv_file):
        """
        Reads waypoints from a CSV file and returns them as a list of Pose objects.
        """
        df = pd.read_csv(csv_file)

        # Tính giá trị nhỏ nhất và lớn nhất của x và y
        x_min, x_max = df['x'].min(), df['x'].max()
        y_min, y_max = df['y'].min(), df['y'].max()
        rospy.loginfo(f"Giá trị của x từ {x_min:.4f} đến {x_max:.4f}")
        rospy.loginfo(f"Giá trị của y từ {y_min:.4f} đến {y_max:.4f}")

        waypoints = []
        for index, row in df.iterrows():
            pose = Pose()
            pose.position.x = row['x'] / 1000.0  # Convert mm to meters
            pose.position.y = row['y'] / 1000.0  # Convert mm to meters
            pose.position.z = row['z'] / 1000.0  # Convert mm to meters

            # Assuming the orientation remains constant, e.g., pointing down to the paper
            quaternion = quaternion_from_euler(0, np.pi, 0)  # Adjust as needed
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            waypoints.append(pose)
        return waypoints

    def move_to_waypoints(self, waypoints):
        """
        Moves the UR3 arm through the given waypoints using Cartesian path.
        """
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.005,       # eef_step - increased number of steps
            0.0          # jump_threshold
        )
        if fraction < 1.0:
            rospy.logwarn(f"Only {fraction*100:.2f}% of the path was planned successfully.")
        # Execute the plan
        result = self.move_group.execute(plan, wait=True)
        if not result:
            rospy.logerr("Path execution failed.")
        else:
            rospy.loginfo("Path execution succeeded.")

if __name__ == '__main__':
    try:
        ur3_controller = UR3Controller()
        waypoints = ur3_controller.read_waypoints('Final_Waypoints_Robot.csv')
        
        # Log waypoints for debugging
        rospy.loginfo(f"Loaded {len(waypoints)} waypoints from 'Final_Waypoints_Robot.csv'.")
        for i, waypoint in enumerate(waypoints):
            # rospy.loginfo(f"Waypoint {i+1}: x={waypoint.position.x:.4f}, y={waypoint.position.y:.4f}, z={waypoint.position.z:.4f}")
            if i != 0:
                rospy.loginfo(waypoint.position.x - waypoints[i-1].position.x)

        # In ra giá trị nhỏ nhất và lớn nhất của x và y
        df = pd.read_csv('Final_Waypoints_Robot.csv')
        x_min, x_max = df['x'].min(), df['x'].max()
        y_min, y_max = df['y'].min(), df['y'].max()
        rospy.loginfo(f"Giá trị của x từ {x_min:.4f} đến {x_max:.4f}")
        rospy.loginfo(f"Giá trị của y từ {y_min:.4f} đến {y_max:.4f}")
        
        # Dừng 5 giây trước khi tiếp tục
        time.sleep(5)
        
        # ur3_controller.go_to_joint_angle([0,0,0,0,0,0])
        ur3_controller.move_to_waypoints(waypoints)
        rospy.loginfo("Path execution complete.")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()



# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from geometry_msgs.msg import Pose
# import numpy as np
# import tf.transformations

# import csv
# from collections import namedtuple
# import pandas as pd

# class UR3eController:
#     def __init__(self):
#         # Define initial orientation
#         self.start_orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)

#     def read_csv(self):
#         # Define a namedtuple to represent a Point
#         Point = namedtuple('Point', ['x', 'y', 'z'])

#         # Read the CSV file into a DataFrame
#         df = pd.read_csv('Final_Waypoints_Robot.csv', engine='python')

#         # Convert the DataFrame to a list of Point namedtuples
#         points = [Point(*row) for row in df.values]

#         return points

#     def move_ur3e_by_pose(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_ur3e_node', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "manipulator"
#         move_group = moveit_commander.MoveGroupCommander(group_name)

#         pose_goal = geometry_msgs.msg.Pose()
#         pose_goal.orientation.w = 1.0
#         pose_goal.position.x = 0.4
#         pose_goal.position.y = 0.1
#         pose_goal.position.z = 0.2

#         move_group.set_pose_target(pose_goal)

#         plan = move_group.go(wait=True)
#         move_group.stop()
#         move_group.clear_pose_targets()

#         rospy.sleep(5)

#     def move_ur3e_by_joints(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_ur3e_by_joints_node', anonymous=True)

#         robot = moveit_commander.RobotCommander()
#         scene = moveit_commander.PlanningSceneInterface()
#         group_name = "manipulator"
#         move_group = moveit_commander.MoveGroupCommander(group_name)

#         joint_goal = move_group.get_current_joint_values()
#         joint_goal[0] = 0
#         joint_goal[1] = -90*3.14/180
#         joint_goal[2] = 56*3.14/180
#         joint_goal[3] = -94*3.14/180
#         joint_goal[4] = -98*3.14/180
#         joint_goal[5] = 25*3.14/180

#         move_group.go(joint_goal, wait=True)
#         move_group.stop()

#         rospy.sleep(5)

#     def move_ur3e_cartesian_path(self, waypoints, fraction=1.0, maxVelocity=0.05):
#         """
#         Move the UR3e robot arm along a specified Cartesian path.

#         Parameters:
#         - waypoints: A list of geometry_msgs/Pose objects defining the path points.
#         - fraction: Desired fraction of the path to execute (default is 1.0, full path).
#         - maxVelocity: Maximum velocity scaling factor for the arm movement.
#         """
#         # Initialize moveit_commander if not already done
#         moveit_commander.roscpp_initialize(sys.argv)

#         # Initialize the node, planning scene, and robot commander
#         rospy.init_node('ur3e_cartesian_path_node', anonymous=True)
#         scene = moveit_commander.PlanningSceneInterface()
#         robot = moveit_commander.RobotCommander()
#         arm = moveit_commander.MoveGroupCommander("manipulator")

#         # Set the robot reference frame and end effector
#         arm.set_pose_reference_frame("base_link")
#         arm.set_end_effector_link("flange")

#         # Set the maximum velocity scaling factor for the robot arm
#         arm.set_max_velocity_scaling_factor(maxVelocity)

#         # Plan the Cartesian path connecting the waypoints
#         (plan, fraction) = arm.compute_cartesian_path(
#                                  waypoints,   # waypoints to follow
#                                  0.01,        # eef_step, resolution of 1 cm
#                                  0.0,         # jump_threshold, disabled
#                                  False)       # avoid_collisions

#         # Execute the plan to move along the computed Cartesian path
#         if fraction >= 0.9:
#             rospy.loginfo("Path computed successfully. Moving the arm.")
#             arm.execute(plan)
#             rospy.loginfo("Path execution complete.")
#         else:
#             rospy.loginfo("Path planning failed with only {}% achieved".format(fraction * 100))

#         return fraction

# if __name__ == '__main__':
#     try:
#         # Initialize UR3eController object
#         ur3e_controller = UR3eController()

#         ## Control the robot by pose
#         # ur3e_controller.move_ur3e_by_pose()

#         ## Control the robot by joint state
#         # ur3e_controller.move_ur3e_by_joints()

#         # Control the robot by way points
#         # Define the example way points

#         # Test >1000 point
#         waypoints = []
#         point_list = ur3e_controller.read_csv()

#         count = 0

#         # Convert from Quaternion to rotation matrix
#         rot_matrix = tf.transformations.quaternion_matrix([ur3e_controller.start_orientation.x,
#                                                            ur3e_controller.start_orientation.y,
#                                                            ur3e_controller.start_orientation.z,
#                                                            ur3e_controller.start_orientation.w])

#         for i, point in enumerate(point_list):
#             if count % 2 == 0:
#                 pose = Pose()
#                 pose.position.x = round(point.x, 3) / 1000
#                 pose.position.y = round(point.y, 3) / 1000
#                 pose.position.z = round(point.z, 3) / 1000

#                 # Thiết lập định hướng cho waypoint
#                 pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_matrix(rot_matrix))

#                 waypoints.append(pose)

#             if count == 20:
#                 break
#             count += 1

#         for pose in waypoints:
#             print("Pose:", pose)

#         # Call the move_ur3e_cartesian_path method on the ur3e_controller object
#         ur3e_controller.move_ur3e_cartesian_path(waypoints)

#         # Test private
#         # ur3e_controller.move_ur3e_by_joints()

#     except rospy.ROSInterruptException:
#         pass
    