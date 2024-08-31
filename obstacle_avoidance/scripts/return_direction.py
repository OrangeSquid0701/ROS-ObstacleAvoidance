#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class ReturnToDirection:
    def __init__(self):
        rospy.init_node('return_to_direction', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.finished_adjustment_pub = rospy.Publisher('/finished_adjustment', Bool, queue_size=10)

        # Subscribers
        self.return_to_dir_flag_sub = rospy.Subscriber('/return_to_direction_flag', Bool, self.return_to_direction_flag_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.obstacle_detected_sub = rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_detected_callback)

        # Internal state variables
        self.original_yaw = None
        self.current_yaw = None
        self.obstacle_present = False
        self.returning_to_direction = False

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        rospy.loginfo(f"Current yaw: {self.current_yaw}")

        if self.returning_to_direction and self.original_yaw is not None:
            rospy.loginfo("Adjusting position: Returning to original direction.")
            self.return_to_original_direction()


    def obstacle_detected_callback(self, msg):
        rospy.loginfo(f"Obstacle detected callback: Obstacle present = {msg.data}")
        if msg.data and not self.obstacle_present:
            self.obstacle_present = True
            self.original_yaw = self.current_yaw
            rospy.loginfo("Obstacle detected: Storing current yaw as original direction.")

        elif not msg.data and self.obstacle_present:
            self.obstacle_present = False
            self.returning_to_direction = True
            rospy.loginfo("Obstacle cleared: Starting to return to original direction.")

    def return_to_original_direction(self):
        twist = Twist()
        yaw_error = self.original_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))  # Normalize the yaw error

        rospy.loginfo(f"Return to original direction: Yaw error = {yaw_error:.2f}")

        if abs(yaw_error) > 0.1:
            twist.angular.z = 0.5 * yaw_error / abs(yaw_error)
            rospy.loginfo(f"Yaw error: {yaw_error:.2f}. Rotating to correct direction.")
        else:
            twist.angular.z = 0.0
            self.returning_to_direction = False
            rospy.loginfo("Returned to original direction. Stopping rotation.")

            # Publish that adjustment is complete
            finished_msg = Bool()
            finished_msg.data = True
            rospy.loginfo("Publishing finished_adjustment message")
            self.finished_adjustment_pub.publish(finished_msg)
            rospy.loginfo("Finished adjustment: Flag set to True.")

        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Publishing velocity command: Linear = {twist.linear.x}, Angular = {twist.angular.z}")


    def return_to_direction_flag_callback(self, msg):
        if msg.data:
            self.returning_to_direction = True
            rospy.loginfo("Return to direction flag activated.")
        else:
            self.returning_to_direction = False
            rospy.loginfo("Return to direction flag deactivated.")

if __name__ == '__main__':
    rtd = ReturnToDirection()
    rospy.spin()
