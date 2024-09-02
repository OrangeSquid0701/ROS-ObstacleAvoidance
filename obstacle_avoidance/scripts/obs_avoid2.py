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

        # Subscriber to the point cloud topic and obstacle detected flag
        self.point_cloud_sub = rospy.Subscriber('/realsense/points', PointCloud2, self.point_cloud_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_detected_callback)

        # Internal state
        self.latest_point_cloud = None
        self.obstacle_distance_threshold = 2.5  # Example threshold
        self.obstacle_present = False
        self.twist = Twist()
        self.direction_flag = None  # Track current direction

    def obstacle_detected_callback(self, msg):
        # Update obstacle presence state
        self.obstacle_present = msg.data

    def point_cloud_callback(self, point_cloud_msg):
        # Update the latest point cloud
        self.latest_point_cloud = point_cloud_msg

        # Process the latest point cloud if an obstacle is detected
        if self.obstacle_present:
            self.avoid_obstacle_point_cloud()

    def avoid_obstacle_point_cloud(self):
        if self.latest_point_cloud is None:
            return

        pc_array = np.array(list(pc2.read_points(self.latest_point_cloud, skip_nans=True, field_names=("x", "y", "z"))))

        # Segment the point cloud
        left_mask = pc_array[:, 0] < -0.5
        right_mask = pc_array[:, 0] > 0.5
        center_mask = ~left_mask & ~right_mask

        left_section = pc_array[left_mask]
        center_section = pc_array[center_mask]
        right_section = pc_array[right_mask]

        avg_left_dist = np.mean(left_section[:, 2]) if left_section.size else float('inf')
        avg_center_dist = np.mean(center_section[:, 2]) if center_section.size else float('inf')
        avg_right_dist = np.mean(right_section[:, 2]) if right_section.size else float('inf')

        # Adjusted linear and angular sensitivity factors
        linear_sensitivity = 0.2  # Sensitivity factor for linear speed
        angular_sensitivity = 1.8  # Sensitivity factor for turning

        # Desired speeds based on obstacle distances
        target_linear_speed = 0.5  # Default speed when no obstacles
        target_angular_speed = 0.0  # Default no rotation

        if avg_center_dist < self.obstacle_distance_threshold:
            # If there's an obstacle, reduce speed and adjust direction
            target_linear_speed = linear_sensitivity * max(0.05, avg_center_dist / self.obstacle_distance_threshold)
            
            if avg_left_dist > avg_right_dist:
                target_angular_speed = angular_sensitivity * (1.0 / avg_center_dist)  # Turn right
            else:
                target_angular_speed = -angular_sensitivity * (1.0 / avg_center_dist)  # Turn left

        # Smoothly adjust current linear speed towards the target speed
        self.twist.linear.x += (target_linear_speed - self.twist.linear.x) * 0.1  # Adjust the smoothing factor (e.g., 0.1) as needed
        # Smoothly adjust current angular speed towards the target speed
        self.twist.angular.z += (target_angular_speed - self.twist.angular.z) * 0.1  # Adjust the smoothing factor (e.g., 0.1) as needed

        # Print the velocities for debugging
        rospy.loginfo(f"Linear Velocity: {self.twist.linear.x} m/s, Angular Velocity: {self.twist.angular.z} rad/s")

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.twist)


if __name__ == '__main__':
    try:
        oa = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
