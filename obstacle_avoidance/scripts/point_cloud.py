#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import cv2
import tf
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('realsense_pointcloud_publisher', anonymous=True)
    pub = rospy.Publisher('/realsense/points', PointCloud2, queue_size=10)
    
    # Initialize the TF broadcaster
    br = tf2_ros.TransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "camera_link"
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    # Configure depth streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 240, rs.format.z16, 10)

    # Start the pipeline
    try:
        pipeline.start(config)
    except RuntimeError as e:
        rospy.logerr(f"Failed to start pipeline: {e}")
        return

    max_depth = 2.0  # Maximum depth in meters

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = cv2.flip(depth_image, 0)
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            points = []

            for v in range(depth_image.shape[0]):
                for u in range(depth_image.shape[1]):
                    z = depth_image[v, u] * 0.001  # Convert mm to meters
                    if z == 0 or z > max_depth:
                        continue
                    x = (u - intrinsics.ppx) * z / intrinsics.fx
                    y = (v - intrinsics.ppy) * z / intrinsics.fy
                    points.append([x, y, z])

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_link"
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]
            pc2 = point_cloud2.create_cloud(header, fields, points)
            pub.publish(pc2)

            # Broadcast the static transform
            br.sendTransform(static_transformStamped)

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            cv2.imshow('Depth Image', depth_colormap)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        pipeline.stop()

if __name__ == "__main__":
    main()

