#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoidance', anonymous=True)

        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.return_to_dir_pub = rospy.Publisher('/return_to_direction_flag', Bool, queue_size=10)

        # Subscriber to the point cloud topic, return to direction flag, and obstacle detected flag
        self.point_cloud_sub = rospy.Subscriber('/realsense/points', PointCloud2, self.point_cloud_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_detected_callback)
        self.return_to_dir_sub = rospy.Subscriber('/return_to_direction_flag', Bool, self.return_to_direction_flag_callback)

        # Internal state
        self.latest_point_cloud = None
        self.obstacle_distance_threshold = 1.5  # Example threshold
        self.obstacle_present = False
        self.twist = Twist()
        self.returning_to_direction = False

    def return_to_direction_flag_callback(self, msg):
        # Update the returning_to_direction state based on the flag from the ReturnToDirection node
        self.returning_to_direction = msg.data

    def obstacle_detected_callback(self, msg):
        # Update obstacle presence state
        self.obstacle_present = msg.data

    def point_cloud_callback(self, point_cloud_msg):
        # Update the latest point cloud
        self.latest_point_cloud = point_cloud_msg

        # Process the latest point cloud
        if self.latest_point_cloud is not None:
            self.avoid_obstacle_point_cloud()

    def avoid_obstacle_point_cloud(self):
        # Check if there is a point cloud to process
        if self.latest_point_cloud is None:
            rospy.logwarn("No point cloud data available.")
            return

        # Don't send velocity commands if returning to the original direction
        if self.returning_to_direction:
            rospy.loginfo("Returning to original direction, obstacle avoidance is paused.")
            return

        # Transform point cloud into numpy array
        try:
            pc_array = np.array(list(pc2.read_points(self.latest_point_cloud, skip_nans=True, field_names=("x", "y", "z"))))
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
            return

        # Check the shape of pc_array
        rospy.loginfo(f"Point cloud array shape: {pc_array.shape}")

        if pc_array.size == 0:
            rospy.logwarn("Point cloud array is empty.")
            return

        # Ensure that pc_array has the expected number of dimensions
        if pc_array.ndim != 2 or pc_array.shape[1] != 3:
            rospy.logerr("Unexpected point cloud array shape. Expected shape: (N, 3).")
            return

        # Segment the point cloud
        try:
            left_mask = pc_array[:, 0] < -0.5
            right_mask = pc_array[:, 0] > 0.5
            center_mask = ~left_mask & ~right_mask

            left_section = pc_array[left_mask]
            center_section = pc_array[center_mask]
            right_section = pc_array[right_mask]

            avg_left_dist = np.mean(left_section[:, 2]) if left_section.size else float('inf')
            avg_center_dist = np.mean(center_section[:, 2]) if center_section.size else float('inf')
            avg_right_dist = np.mean(right_section[:, 2]) if right_section.size else float('inf')

            scale_factor = min(avg_center_dist / self.obstacle_distance_threshold, 1.0)
            scale_factor = max(scale_factor, 0.3)

            if avg_center_dist < self.obstacle_distance_threshold:
                if avg_left_dist > avg_right_dist:
                    self.twist.linear.x = 0.5 * scale_factor
                    self.twist.angular.z = 1.5 * (1.0 - scale_factor)
                else:
                    self.twist.linear.x = 0.5 * scale_factor
                    self.twist.angular.z = -1.5 * (1.0 - scale_factor)

            # Print the velocities
            rospy.loginfo(f"Linear Velocity: {self.twist.linear.x} m/s, Angular Velocity: {self.twist.angular.z} rad/s")

            # Publish the velocity command
            self.cmd_vel_pub.publish(self.twist)
        except Exception as e:
            rospy.logerr(f"Error during obstacle avoidance: {e}")

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
