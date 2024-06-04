#!/usr/bin/env python
import rospy
import moveit_commander
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
import sys
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
        self.move_group.set_max_velocity_scaling_factor(0.2)  # Increase the speed
        self.move_group.set_max_acceleration_scaling_factor(0.2)  # Increase the acceleration
        
        self.marker_pub = rospy.Publisher('/move_group/display_contacts', MarkerArray, queue_size=100)
        self.marker_array = MarkerArray()
        self.marker_id = 0
        
    def read_waypoints(self, csv_file):
        if not os.path.isfile(csv_file):
            rospy.logerr(f"File not found: {csv_file}")
            return []
        df = pd.read_csv(csv_file)
        waypoints = []
        x_prev, y_prev, z_prev = None, None, None
        for _, row in df.iterrows():
            if row['x'] == x_prev and row['y'] == y_prev and row['z'] == z_prev:
                continue
            else:
                if row['z'] == 0.8 or row['z'] == 1.8:
                    pose = Pose()
                    pose.position.x = (row['x'] + 100) / 1000.0  # Shift x value to be within the workspace center
                    pose.position.y = (row['y'] + 145) / 1000.0  # Shift y value to be within the workspace center
                    pose.position.z = (row['z']) / 1000.0  # Keep z value if it is reasonable
                    quaternion = quaternion_from_euler(0, np.pi, 0)
                    pose.orientation.x = quaternion[0]
                    pose.orientation.y = quaternion[1]
                    pose.orientation.z = quaternion[2]
                    pose.orientation.w = quaternion[3]
                    # Check if the waypoint is within the workspace
                    if self.is_within_workspace(pose):
                        waypoints.append(pose)
                    x_prev, y_prev, z_prev = row['x'], row['y'], row['z']
        return waypoints

    def is_within_workspace(self, pose):
        # Define the workspace boundaries
        min_x, max_x = 0.0, 0.5  # Example values, adjust as needed
        min_y, max_y = -0.5, 0.5
        min_z, max_z = 0.0, 0.5
        if (min_x <= pose.position.x <= max_x and
            min_y <= pose.position.y <= max_y and
            min_z <= pose.position.z <= max_z):
            return True
        return False
    
    def create_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.0005
        marker.scale.y = 0.0005
        marker.scale.z = 0.0005
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.id = self.marker_id
        self.marker_id += 1
        self.marker_array.markers.append(marker)
        self.publish_markers()
        rospy.sleep(0.1)

    def publish_markers(self):
        self.marker_pub.publish(self.marker_array)

    def move_to_waypoints(self, waypoints, start_index, num_points):
        if start_index >= len(waypoints):
            rospy.loginfo("All waypoints have been executed.")
            return
        # Compute the Cartesian path
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints[start_index:start_index + num_points],
            0.01,  # eef_step (meters)
            0.0    # jump_threshold (radians)
        )
        if fraction > 0.9:  # Adjust the fraction threshold to be less strict
            result = self.move_group.execute(plan, wait=True)
            if result:
                for i in range(start_index, start_index + num_points):
                    try:
                        pose = waypoints[i]
                        self.create_marker(pose.position.x, pose.position.y, pose.position.z)
                        self.publish_markers()  # Publish marker after reaching the waypoint
                    except:
                        continue
                rospy.loginfo(f"Executed waypoints from index {start_index} to {start_index + num_points} with fraction {fraction}")
            else:
                rospy.logwarn(f"Control failed for waypoints from index {start_index} to {start_index + num_points}, fraction: {fraction}")
        else:
            rospy.logwarn(f"Path planning failed for waypoints from index {start_index} to {start_index + num_points}, fraction: {fraction}")
            rospy.logwarn(f"Retrying with smaller set of points.")
            # Retry with smaller set of points
            for i in range(num_points):
                (plan, fraction) = self.move_group.compute_cartesian_path(
                    waypoints[start_index + i:start_index + i + 1],
                    0.01,
                    0.0
                )
                if fraction > 0.9:
                    result = self.move_group.execute(plan, wait=True)
                    if result:
                        rospy.loginfo(f"Executed waypoint at index {start_index + i} with fraction {fraction}")
                    else:
                        rospy.logwarn(f"Control failed for waypoint at index {start_index + i}")
                    

    def move_ur3e_by_joints(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[:6] = [
            86.99 * np.pi / 180,   # Base
            -69.87 * np.pi / 180,  # Shoulder
            68.39 * np.pi / 180,   # Elbow
            -91.17 * np.pi / 180,  # Wrist 1
            -90.77 * np.pi / 180,  # Wrist 2
            88.81 * np.pi / 180    # Wrist 3
        ]
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        rospy.sleep(10)

def reset_moveit():
    moveit_commander.roscpp_shutdown()
    moveit_commander.roscpp_initialize(sys.argv)

if __name__ == '__main__':
    try:
        ur3_controller = UR3Controller()
        waypoints = ur3_controller.read_waypoints('Final_Waypoints_Robot.csv')
        rospy.loginfo(f"Loaded {len(waypoints)} waypoints from 'Final_Waypoints_Robot.csv'.")
        if waypoints:
            ur3_controller.publish_markers()  # Publish markers after reading waypoints
            ur3_controller.move_ur3e_by_joints()
            start_index = 0
            num_points_per_iteration = 20  # Reduce the number of points per iteration
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
