#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.point_cloud_sub = rospy.Subscriber('/realsense/points', PointCloud2, self.point_cloud_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_detected_callback)
        self.latest_point_cloud = None
        self.obstacle_distance_threshold = 1.5  # in meters
        self.obstacle_present = False
        self.movement_phase = None  # None, 'rotating', 'moving_forward'
        self.twist = Twist()
        self.max_linear_velocity = 0.1  # Maximum linear velocity in meters per second
        self.angular_velocity_factor = 2.0  # Factor to increase angular velocity for quick 90-degree turn

    def obstacle_detected_callback(self, msg):
        # Update the presence of an obstacle
        self.obstacle_present = msg.data
        rospy.loginfo(f"Obstacle detected presence updated: {self.obstacle_present}")
        if self.obstacle_present and self.movement_phase is None:
            self.start_rotation()

    def point_cloud_callback(self, point_cloud_msg):
        # Store the latest point cloud
        self.latest_point_cloud = point_cloud_msg

    def start_rotation(self):
        # Start rotating 90 degrees
        self.movement_phase = 'rotating'
        angle_direction = 1  # Positive for left, negative for right, choose based on your system setup
        self.twist.angular.z = angle_direction * self.angular_velocity_factor
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        # Schedule stopping the rotation and starting forward movement
        rospy.Timer(rospy.Duration(0.5), self.stop_rotation, oneshot=True)

    def stop_rotation(self, event):
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        self.start_moving_forward()

    def start_moving_forward(self):
        # Move forward after rotation
        self.movement_phase = 'moving_forward'
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
        # Schedule stopping the forward movement
        rospy.Timer(rospy.Duration(1), self.stop_moving_forward, oneshot=True)

    def stop_moving_forward(self, event):
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.movement_phase = None  # Reset phase

    def calculate_and_publish_velocities(self, pc_array):
        # This function remains for additional custom logic if needed
        pass

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()

