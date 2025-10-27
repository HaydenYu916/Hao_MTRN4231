#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
叶子检测坐标到机械臂坐标系转换节点

功能:
- 订阅叶子检测话题 (/leaf_detection/coordinates)
- 使用TF2将相机坐标转换到机械臂基础坐标系
- 发布转换后的目标位置
- 支持与MoveIt集成以执行抓取动作
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json
import numpy as np
import cv2
import pyrealsense2 as rs


class LeafToArmTransformer(Node):
    """
    将叶子检测坐标转换为机械臂坐标系的节点
    """
    
    def __init__(self):
        super().__init__('leaf_to_arm_transformer')
        
        # TF2 设置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # RealSense 相机内参
        self.intrinsics = None
        self.depth_image = None
        self.cv_bridge = CvBridge()
        
        # 订阅叶子检测话题
        self.leaf_coords_subscription = self.create_subscription(
            String,
            '/leaf_detection/coordinates',
            self.leaf_coordinates_callback,
            10
        )
        
        # 订阅深度图像
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        # 订阅相机信息（获取内参）
        from sensor_msgs.msg import CameraInfo
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布转换后的目标位置
        self.arm_target_publisher = self.create_publisher(
            PoseStamped,
            '/leaf_detection/arm_target',
            10
        )
        
        # 发布可视化标记
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/leaf_detection/arm_target_markers',
            10
        )
        
        # 参数配置
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('arm_base_frame', 'base_link')
        self.declare_parameter('depth_offset', 0.038)  # 相机到机械臂的偏移 (m)
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.depth_offset = self.get_parameter('depth_offset').value
        
        self.get_logger().info(
            f'🌿 叶子到机械臂坐标转换节点已启动\n'
            f'  相机坐标系: {self.camera_frame}\n'
            f'  机械臂基础框架: {self.arm_base_frame}\n'
            f'  深度偏移: {self.depth_offset}m'
        )
        
        self.marker_id = 0
    
    def camera_info_callback(self, msg):
        """获取相机内参"""
        if self.intrinsics is not None:
            return
        
        try:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]  # 主点x
            self.intrinsics.ppy = msg.k[5]  # 主点y
            self.intrinsics.fx = msg.k[0]   # 焦距x
            self.intrinsics.fy = msg.k[4]   # 焦距y
            
            # 处理畸变系数
            if msg.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif msg.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            
            self.intrinsics.coeffs = [i for i in msg.d]
            
            self.get_logger().info('✓ 相机内参已获取')
        except Exception as e:
            self.get_logger().error(f'✗ 获取相机内参失败: {str(e)}')
    
    def depth_callback(self, msg):
        """获取深度图像"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f'✗ 深度图像转换失败: {str(e)}')
    
    def pixel_to_3d(self, pixel_x, pixel_y):
        """
        将像素坐标转换为3D坐标（相机坐标系）
        
        Args:
            pixel_x, pixel_y: 像素坐标
            
        Returns:
            [x, y, z]: 相机坐标系中的3D点
        """
        if self.depth_image is None or self.intrinsics is None:
            return None
        
        try:
            # 获取该像素的深度值 (转换为米)
            depth_value = self.depth_image[int(pixel_y), int(pixel_x)] * 0.001
            
            if depth_value <= 0:
                return None
            
            # 使用RealSense的反投影函数
            x, y, z = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                (pixel_x, pixel_y),
                depth_value
            )
            
            return [x, y, z]
        except Exception as e:
            self.get_logger().error(f'✗ 像素转换失败: {str(e)}')
            return None
    
    def transform_to_arm_frame(self, point_camera):
        """
        使用TF2将点从相机坐标系转换到机械臂坐标系
        
        Args:
            point_camera: [x, y, z] 相机坐标系中的点
            
        Returns:
            PoseStamped: 机械臂坐标系中的位置
        """
        try:
            # 创建点的时间戳
            now = self.get_clock().now().to_msg()
            
            # 创建相机坐标系中的点
            point_stamped = PointStamped()
            point_stamped.header.frame_id = self.camera_frame
            point_stamped.header.stamp = now
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # 获取从相机到机械臂的变换
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.camera_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # 变换点
            point_transformed = tf2_ros.do_transform_point(point_stamped, transform)
            
            # 创建PoseStamped消息
            pose = PoseStamped()
            pose.header = point_transformed.header
            pose.pose.position = point_transformed.point
            pose.pose.orientation.w = 1.0  # 默认方向
            
            return pose
            
        except tf2_ros.TransformException as e:
            self.get_logger().warning(f'⚠️  坐标变换失败: {str(e)}')
            return None
        except Exception as e:
            self.get_logger().error(f'✗ 变换处理失败: {str(e)}')
            return None
    
    def leaf_coordinates_callback(self, msg):
        """
        处理叶子检测坐标消息
        """
        try:
            data = json.loads(msg.data)
            coordinates = data.get('coordinates', [])
            
            if not coordinates:
                return
            
            # 处理每个检测到的叶子
            arm_targets = []
            
            for leaf in coordinates:
                leaf_id = leaf.get('id', 0)
                center = leaf.get('center', {})
                
                if not center or 'x' not in center or 'y' not in center:
                    continue
                
                pixel_x = center['x']
                pixel_y = center['y']
                
                # 将像素坐标转换为3D点（相机坐标系）
                point_3d = self.pixel_to_3d(pixel_x, pixel_y)
                
                if point_3d is None:
                    self.get_logger().warning(f'⚠️  叶子 #{leaf_id} 无法获取深度信息')
                    continue
                
                # 应用深度偏移
                point_3d[0] -= self.depth_offset
                
                self.get_logger().info(
                    f'📍 叶子 #{leaf_id}\n'
                    f'  像素坐标: ({pixel_x:.1f}, {pixel_y:.1f})\n'
                    f'  相机坐标: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f}) m'
                )
                
                # 变换到机械臂坐标系
                arm_pose = self.transform_to_arm_frame(point_3d)
                
                if arm_pose is not None:
                    self.get_logger().info(
                        f'  机械臂坐标: ({arm_pose.pose.position.x:.3f}, '
                        f'{arm_pose.pose.position.y:.3f}, '
                        f'{arm_pose.pose.position.z:.3f}) m'
                    )
                    
                    # 发布目标位置
                    self.arm_target_publisher.publish(arm_pose)
                    arm_targets.append((leaf_id, arm_pose))
            
            # 发布可视化标记
            if arm_targets:
                self.publish_target_markers(arm_targets)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'✗ JSON解析失败: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'✗ 回调处理失败: {str(e)}')
    
    def publish_target_markers(self, arm_targets):
        """
        发布可视化标记
        """
        marker_array = MarkerArray()
        
        for leaf_id, pose in arm_targets:
            marker = Marker()
            marker.header.frame_id = self.arm_base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = pose.pose
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker.text = f"Leaf #{leaf_id}"
            
            marker_array.markers.append(marker)
            self.marker_id += 1
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = LeafToArmTransformer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✓ 叶子到机械臂转换节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
