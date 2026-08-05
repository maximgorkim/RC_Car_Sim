#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms for sensor suite
        self.publish_static_transforms()
        
        self.get_logger().info('Static TF Publisher started - Publishing base_link and sensor transforms')
    
    def publish_static_transforms(self):
        transforms = []
        
        # Camera transform (relative to base_link)
        # From model.sdf: <pose>0 0 0.2 0 0.2617 0</pose>
        camera_tf = TransformStamped()
        camera_tf.header.stamp = self.get_clock().now().to_msg()
        camera_tf.header.frame_id = 'base_link'
        camera_tf.child_frame_id = 'camera_link'
        camera_tf.transform.translation.x = 0.0
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.2
        # Quaternion for pitch = 0.2617 rad (15 degrees)
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.1305
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 0.9914
        transforms.append(camera_tf)
        
        # Lidar transform (relative to base_link)
        # From model.sdf: <pose>0 0 0.15 0 0 0</pose>
        lidar_tf = TransformStamped()
        lidar_tf.header.stamp = self.get_clock().now().to_msg()
        lidar_tf.header.frame_id = 'base_link'
        lidar_tf.child_frame_id = 'lidar_link'
        lidar_tf.transform.translation.x = 0.0
        lidar_tf.transform.translation.y = 0.0
        lidar_tf.transform.translation.z = 0.15
        lidar_tf.transform.rotation.x = 0.0
        lidar_tf.transform.rotation.y = 0.0
        lidar_tf.transform.rotation.z = 0.0
        lidar_tf.transform.rotation.w = 1.0
        transforms.append(lidar_tf)
        
        # IMU is in chassis - same as base_link
        imu_tf = TransformStamped()
        imu_tf.header.stamp = self.get_clock().now().to_msg()
        imu_tf.header.frame_id = 'base_link'
        imu_tf.child_frame_id = 'imu_link'
        imu_tf.transform.translation.x = 0.0
        imu_tf.transform.translation.y = 0.0
        imu_tf.transform.translation.z = 0.0
        imu_tf.transform.rotation.x = 0.0
        imu_tf.transform.rotation.y = 0.0
        imu_tf.transform.rotation.z = 0.0
        imu_tf.transform.rotation.w = 1.0
        transforms.append(imu_tf)
        
        # GPS is also in chassis - same as base_link
        gps_tf = TransformStamped()
        gps_tf.header.stamp = self.get_clock().now().to_msg()
        gps_tf.header.frame_id = 'base_link'
        gps_tf.child_frame_id = 'gps_link'
        gps_tf.transform.translation.x = 0.0
        gps_tf.transform.translation.y = 0.0
        gps_tf.transform.translation.z = 0.0
        gps_tf.transform.rotation.x = 0.0
        gps_tf.transform.rotation.y = 0.0
        gps_tf.transform.rotation.z = 0.0
        gps_tf.transform.rotation.w = 1.0
        transforms.append(gps_tf)
        
        # Broadcast all static transforms
        self.tf_static_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
