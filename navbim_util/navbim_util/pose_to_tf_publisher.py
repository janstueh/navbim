#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTfPublisher(Node):
    def __init__(self):
        super().__init__('pose_to_tf_publisher')
        
        # Declare parameters
        self.declare_parameter('start_pose_topic', '/navbim/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        start_pose_topic = self.get_parameter('start_pose_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize transforms
        self.map_to_odom_transform = TransformStamped()
        self.map_to_odom_transform.header.frame_id = self.map_frame
        self.map_to_odom_transform.child_frame_id = self.odom_frame
        
        self.odom_to_base_transform = TransformStamped()
        self.odom_to_base_transform.header.frame_id = self.odom_frame
        self.odom_to_base_transform.child_frame_id = self.base_frame
        # Initialize as identity transform
        self.odom_to_base_transform.transform.rotation.w = 1.0
        
        # Subscribe to corrected initial pose
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            start_pose_topic,
            self.pose_callback,
            10
        )
        
        # Create timer to publish transforms at 50Hz
        self.timer = self.create_timer(0.02, self.publish_transforms)
        
        # Flag to check if we have received an initial pose
        self.has_initial_pose = False
        
        self.get_logger().info(f'Pose to TF publisher started, listening on {start_pose_topic}')

    def pose_callback(self, msg: PoseStamped):
        """Callback for receiving corrected initial pose from BIM panel"""
        
        # Update the map->odom transform based on the corrected initial pose
        self.map_to_odom_transform.header.stamp = self.get_clock().now().to_msg()
        self.map_to_odom_transform.transform.translation.x = msg.pose.position.x
        self.map_to_odom_transform.transform.translation.y = msg.pose.position.y
        self.map_to_odom_transform.transform.translation.z = msg.pose.position.z
        self.map_to_odom_transform.transform.rotation = msg.pose.orientation
        
        # Reset odom->base_link to identity since we're setting the robot's initial position
        self.odom_to_base_transform.header.stamp = self.get_clock().now().to_msg()
        self.odom_to_base_transform.transform.translation.x = 0.0
        self.odom_to_base_transform.transform.translation.y = 0.0
        self.odom_to_base_transform.transform.translation.z = 0.0
        self.odom_to_base_transform.transform.rotation.x = 0.0
        self.odom_to_base_transform.transform.rotation.y = 0.0
        self.odom_to_base_transform.transform.rotation.z = 0.0
        self.odom_to_base_transform.transform.rotation.w = 1.0
        
        self.has_initial_pose = True

    def publish_transforms(self):
        """Publish the transform chain at regular intervals"""
        if self.has_initial_pose:
            # Update timestamps
            now = self.get_clock().now().to_msg()
            self.map_to_odom_transform.header.stamp = now
            self.odom_to_base_transform.header.stamp = now
            
            # Publish both transforms
            self.tf_broadcaster.sendTransform([
                self.map_to_odom_transform,
                self.odom_to_base_transform
            ])


def main(args=None):
    rclpy.init(args=args)
    
    pose_to_tf_publisher = PoseToTfPublisher()
    
    try:
        rclpy.spin(pose_to_tf_publisher)
    except KeyboardInterrupt:
        pass
    
    pose_to_tf_publisher.destroy_node()
    
    # Only shutdown if context is still valid
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()