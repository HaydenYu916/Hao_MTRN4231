#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Coordinate Transform Node
Transform leaf detection results from camera frame to robot base frame
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PointStamped
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
import os
import yaml


class CoordinateTransformer(Node):
    def __init__(self):
        super().__init__('coordinate_transformer')
        
        # ============================================
        # Load camera extrinsic configuration
        # ============================================
        # Get config folder path
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
            'config', 'camera_extrinsics.yaml'
        )
        
        # Read config file
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.camera_rpy = config['camera_rpy']
            self.camera_position = config['camera_position']
            self.get_logger().info(f'Extrinsic config loaded: {config_path}')
        except Exception as e:
            self.get_logger().warn(f'Unable to load extrinsic config: {e}, using defaults')
            # Default values
            self.camera_rpy = [-0.05, 0.30, 1.57]
            self.camera_position = [0.80, -0.10, 0.95]
        
        # Build extrinsic matrix
        self._build_extrinsic_matrix()
        
        # Subscribe to leaf coordinates
        self.subscription = self.create_subscription(
            String,
            '/leaf_detection/coordinates',
            self.coordinate_callback,
            10
        )
        
        # 发布转换后的坐标
        self.transformed_publisher = self.create_publisher(
            String,
            '/leaf_detection/base_coordinates',
            10
        )
        
        # Publish TF transform (camera to robot base)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Periodically publish static TF
        self.tf_timer = self.create_timer(1.0, self.publish_tf)
        
        self.get_logger().info('Coordinate transform node started')
        self.get_logger().info(f'  Camera position: {self.camera_position}')
        self.get_logger().info(f'  Camera RPY: {self.camera_rpy}')
    
    def _build_extrinsic_matrix(self):
        """Build extrinsic matrix T_base_camera"""
        # Generate rotation matrix from Euler angles
        R_bc = R.from_euler('xyz', self.camera_rpy, degrees=False).as_matrix()
        
        # Build homogeneous transformation matrix
        self.T_bc = np.eye(4)
        self.T_bc[:3, :3] = R_bc
        self.T_bc[:3, 3] = self.camera_position
        
        self.get_logger().info(f'Extrinsic matrix:\n{self.T_bc}')
    
    def publish_tf(self):
        """Publish TF transform from camera to robot flange"""
        now = self.get_clock().now().to_msg()
        
        # TF 1: Create base_link (if not exists)
        t_base = TransformStamped()
        t_base.header.stamp = now
        t_base.header.frame_id = "world"  # World frame
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = 0.0
        t_base.transform.translation.y = 0.0
        t_base.transform.translation.z = 0.0
        t_base.transform.rotation.x = 0.0
        t_base.transform.rotation.y = 0.0
        t_base.transform.rotation.z = 0.0
        t_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_base)
        
        # TF 2: base_link -> flange (assuming robot at zero pose, flange above base_link)
        t_flange = TransformStamped()
        t_flange.header.stamp = now
        t_flange.header.frame_id = "base_link"
        t_flange.child_frame_id = "flange"
        # Assume flange about 0.9m above base (for UR5e)
        t_flange.transform.translation.x = 0.0
        t_flange.transform.translation.y = 0.0
        t_flange.transform.translation.z = 0.9  # estimated
        t_flange.transform.rotation.x = 0.0
        t_flange.transform.rotation.y = 0.0
        t_flange.transform.rotation.z = 0.0
        t_flange.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_flange)
        
        # TF 3: flange -> camera_color_optical_frame (simplified: direct connection, skip camera_link)
        t_camera = TransformStamped()
        t_camera.header.stamp = now
        t_camera.header.frame_id = "flange"
        t_camera.child_frame_id = "camera_color_optical_frame"
        
        # Set translation (transform from flange to camera)
        t_camera.transform.translation.x = float(self.camera_position[0])
        t_camera.transform.translation.y = float(self.camera_position[1])
        t_camera.transform.translation.z = float(self.camera_position[2])
        
        # Set rotation (convert from Euler angles to quaternion)
        quat = R.from_euler('xyz', self.camera_rpy, degrees=False).as_quat()
        t_camera.transform.rotation.x = float(quat[0])
        t_camera.transform.rotation.y = float(quat[1])
        t_camera.transform.rotation.z = float(quat[2])
        t_camera.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t_camera)
    
    def camera_to_base(self, point_camera):
        """
        Transform point from camera frame to robot base frame
        
        Args:
            point_camera: Point in camera frame [X, Y, Z] (meters)
        
        Returns:
            Point in robot base frame [X, Y, Z] (meters)
        """
        # Convert to homogeneous coordinates
        Pc = np.array([point_camera[0], point_camera[1], point_camera[2], 1.0])
        
        # Coordinate transform
        Pb = self.T_bc @ Pc
        
        return Pb[:3].tolist()
    
    def coordinate_callback(self, msg):
        """Process leaf coordinate message"""
        try:
            data = json.loads(msg.data)
            
            # Transform all leaf coordinates
            transformed_data = data.copy()
            transformed_coordinates = []
            
            for leaf in data.get('coordinates', []):
                point_3d = leaf.get('point_3d')
                
                if point_3d is not None:
                    # Point in camera frame
                    Pc = point_3d
                    
                    # Transform to robot base frame
                    Pb = self.camera_to_base(Pc)
                    
                    # Save transformed coordinates
                    transformed_leaf = leaf.copy()
                    transformed_leaf['base_coordinates'] = Pb  # New: coordinates in base frame
                    transformed_coordinates.append(transformed_leaf)
                else:
                    # If no 3D coordinates, keep unchanged
                    transformed_leaf = leaf.copy()
                    transformed_leaf['base_coordinates'] = None
                    transformed_coordinates.append(transformed_leaf)
            
            transformed_data['coordinates'] = transformed_coordinates
            
            # Publish transformed coordinates
            output_msg = String()
            output_msg.data = json.dumps(transformed_data)
            self.transformed_publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().error(f'Coordinate transform error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CoordinateTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nCoordinate transform node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

