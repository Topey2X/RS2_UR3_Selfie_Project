#!/usr/bin/env python3
import rospy
import moveit_commander
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import sys
import time
import os

class UR3Controller:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_ur3_controller', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.move_group.set_pose_reference_frame("base_link")
        self.move_group.set_goal_position_tolerance(0.01)
        self.move_group.set_goal_orientation_tolerance(0.01)

    def read_waypoints(self, csv_file):
        if not os.path.isfile(csv_file):
            rospy.logerr(f"File not found: {csv_file}")
            return []
        df = pd.read_csv(csv_file)
        waypoints = []
        x = 0.0
        y = 0.0
        z = 0.0
        for _, row in df.iterrows():
            if row['x'] == x and row['y'] == y and row['z'] == z: 
                continue
            else:
                pose = Pose()
                pose.position.x = (row['x']+100) / 1000.0
                pose.position.y = (row['y']+100) / 1000.0
                pose.position.z = row['z'] / 1000.0
                quaternion = quaternion_from_euler(0, np.pi, 0)
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                waypoints.append(pose)
                x = row['x']
                y = row['y']
                z = row['z']
        return waypoints

    def move_to_waypoints(self, waypoints, start_index, num_points):
        if start_index >= len(waypoints):
            rospy.loginfo("All waypoints have been executed.")
            return
        end_index = min(start_index + num_points, len(waypoints))
        rospy.loginfo(f"Executing from point {start_index} to point {end_index}.")
        waypoints_subset = waypoints[start_index:end_index]
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints_subset, 0.5, 0.0)
        rospy.sleep(1)
        if fraction < 1.0:
            rospy.logwarn(f"Only {fraction*100:.2f}% of the path was planned successfully.")
        if not self.move_group.execute(plan, wait=True):
            rospy.logerr("Path execution failed.")
        else:
            rospy.loginfo("Path execution succeeded.")

    def move_ur3e_by_joints(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[:6] = [0, -45*np.pi/180, 56*np.pi/180, -94*np.pi/180, -98*np.pi/180, 25*np.pi/180]
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        rospy.sleep(10)

if __name__ == '__main__':
    try:
        ur3_controller = UR3Controller()
        waypoints = ur3_controller.read_waypoints('scripts/Final_Waypoints_Robot.csv')
        rospy.loginfo(f"Loaded {len(waypoints)} waypoints from 'Final_Waypoints_Robot.csv'.")
        if waypoints:
            ur3_controller.move_ur3e_by_joints()
            start_index = 0
            num_points_per_iteration = 50
            while start_index < len(waypoints):
                ur3_controller.move_to_waypoints(waypoints, start_index, num_points_per_iteration)
                start_index += num_points_per_iteration
            rospy.loginfo("Path execution complete.")
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        ur3_controller.move_group.stop()
        moveit_commander.roscpp_shutdown()


# #!/usr/bin/env python3
# import rospy
# import moveit_commander
# import pandas as pd
# import numpy as np
# from geometry_msgs.msg import Pose
# from tf.transformations import quaternion_from_euler
# import sys
# import time
# import os

# class UR3Controller:
#     def __init__(self):
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node('move_ur3_controller', anonymous=True)
#         self.robot = moveit_commander.RobotCommander()
#         self.scene = moveit_commander.PlanningSceneInterface()
#         self.move_group = moveit_commander.MoveGroupCommander("manipulator")
#         self.move_group.set_pose_reference_frame("base_link")
#         self.move_group.set_goal_position_tolerance(0.01)
#         self.move_group.set_goal_orientation_tolerance(0.01)

#     def read_waypoints(self, csv_file):
#         if not os.path.isfile(csv_file):
#             rospy.logerr(f"File not found: {csv_file}")
#             return []
#         df = pd.read_csv(csv_file)
#         waypoints = []
#         x = 0.0
#         y = 0.0
#         z = 0.0
#         for _, row in df.iterrows():
#             # if row['x'] == x and row['y'] == y and row['z'] == z: 
#             #     continue
#             # else:
#                 pose = Pose()
#                 pose.position.x = (row['x']+100) / 1000.0
#                 pose.position.y = (row['y']+100) / 1000.0
#                 pose.position.z = row['z'] / 1000.0
#                 quaternion = quaternion_from_euler(0, np.pi, 0)
#                 pose.orientation.x = quaternion[0]
#                 pose.orientation.y = quaternion[1]
#                 pose.orientation.z = quaternion[2]
#                 pose.orientation.w = quaternion[3]
#                 waypoints.append(pose)
#                 x = row['x']
#                 y = row['y']
#                 z = row['z']
#         return waypoints

#     def move_to_waypoints(self, waypoints, start_index, num_points):
#         if start_index >= len(waypoints):
#             rospy.loginfo("All waypoints have been executed.")
#             return
#         end_index = min(start_index + num_points, len(waypoints))
#         waypoints_subset = waypoints[start_index:end_index]
#         (plan, fraction) = self.move_group.compute_cartesian_path(waypoints_subset, 0.5, 0.0)
#         rospy.sleep(1)
#         if fraction < 1.0:
#             rospy.logwarn(f"Only {fraction*100:.2f}% of the path was planned successfully.")
#         if not self.move_group.execute(plan, wait=True):
#             rospy.logerr("Path execution failed.")
#         else:
#             rospy.loginfo("Path execution succeeded.")

#     def move_ur3e_by_joints(self):
#         joint_goal = self.move_group.get_current_joint_values()
#         joint_goal[:6] = [0, -45*np.pi/180, 56*np.pi/180, -94*np.pi/180, -98*np.pi/180, 25*np.pi/180]
#         self.move_group.go(joint_goal, wait=True)
#         self.move_group.stop()
#         rospy.sleep(10)

# if __name__ == '__main__':
#     try:
#         ur3_controller = UR3Controller()
#         waypoints = ur3_controller.read_waypoints('scripts/Final_Waypoints_Robot.csv')
#         rospy.loginfo(f"Loaded {len(waypoints)} waypoints from 'Final_Waypoints_Robot.csv'.")
#         if waypoints:
#             ur3_controller.move_ur3e_by_joints()
#             start_index = 0
#             num_points_per_iteration = 50
#             while start_index < len(waypoints):
#                 ur3_controller.move_to_waypoints(waypoints, start_index, num_points_per_iteration)
#                 start_index += num_points_per_iteration
#             rospy.loginfo("Path execution complete.")
#     except rospy.ROSInterruptException:
#         pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ur3_controller.move_group.stop()
#         moveit_commander.roscpp_shutdown()