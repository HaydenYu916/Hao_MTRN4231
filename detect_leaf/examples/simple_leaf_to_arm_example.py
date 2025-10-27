#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单示例：将叶子检测坐标转换到机械臂坐标系

这是一个独立的示例脚本，演示核心的坐标转换过程。
可以用于测试和理解整个转换流程。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import json
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np


class SimpleLeafToArmExample(Node):
    """简单的叶子到机械臂坐标转换示例"""
    
    def __init__(self):
        super().__init__('simple_leaf_to_arm_example')
        
        # TF2设置
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 相机参数
        self.intrinsics = None
        self.depth_image = None
        self.cv_bridge = CvBridge()
        
        # 订阅信息
        self.create_subscription(
            String,
            '/leaf_detection/coordinates',
            self.on_leaf_detected,
            10
        )
        
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.on_depth_image,
            10
        )
        
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.on_camera_info,
            10
        )
        
        # 发布转换结果
        self.arm_pose_pub = self.create_publisher(
            PoseStamped,
            '/leaf_to_arm_result',
            10
        )
        
        self.get_logger().info('🌿 简单转换示例已启动')
    
    def on_camera_info(self, msg):
        """获取相机内参"""
        if self.intrinsics is not None:
            return
        
        try:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = [i for i in msg.d]
            
            print(f"✓ 相机内参已获取")
            print(f"  分辨率: {msg.width}x{msg.height}")
            print(f"  焦距: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}")
            print(f"  主点: cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}")
        except Exception as e:
            self.get_logger().error(f'获取相机内参失败: {e}')
    
    def on_depth_image(self, msg):
        """获取深度图像"""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f'深度图转换失败: {e}')
    
    def pixel_to_3d(self, pixel_x, pixel_y):
        """将像素坐标转换为3D点"""
        if self.depth_image is None or self.intrinsics is None:
            print("✗ 深度图或内参未就绪")
            return None
        
        try:
            # 获取深度值（单位：mm -> m）
            depth_mm = self.depth_image[int(pixel_y), int(pixel_x)]
            depth_m = depth_mm * 0.001
            
            if depth_m <= 0:
                print(f"✗ 无效的深度值: {depth_mm}mm")
                return None
            
            # 反投影
            x, y, z = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                (pixel_x, pixel_y),
                depth_m
            )
            
            print(f"  像素 ({pixel_x}, {pixel_y}) -> 相机坐标 ({x:.3f}, {y:.3f}, {z:.3f})m")
            return [x, y, z]
            
        except Exception as e:
            self.get_logger().error(f'像素转换失败: {e}')
            return None
    
    def transform_to_arm(self, point_3d):
        """使用TF2变换到机械臂坐标系"""
        try:
            # 创建带时间戳的点
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_color_optical_frame'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = point_3d[0]
            point_stamped.point.y = point_3d[1]
            point_stamped.point.z = point_3d[2]
            
            # 获取变换
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_color_optical_frame',
                point_stamped.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 应用变换
            point_arm = tf2_ros.do_transform_point(point_stamped, transform)
            
            print(f"  机械臂坐标: ({point_arm.point.x:.3f}, {point_arm.point.y:.3f}, {point_arm.point.z:.3f})m")
            
            # 创建PoseStamped消息
            pose = PoseStamped()
            pose.header = point_arm.header
            pose.pose.position = point_arm.point
            pose.pose.orientation.w = 1.0
            
            return pose
            
        except tf2_ros.TransformException as e:
            print(f"✗ TF变换失败: {e}")
            print("  确保机械臂驱动已启动")
            return None
    
    def on_leaf_detected(self, msg):
        """处理叶子检测消息"""
        try:
            data = json.loads(msg.data)
            coordinates = data.get('coordinates', [])
            
            if not coordinates:
                return
            
            # 处理第一个叶子
            leaf = coordinates[0]
            center = leaf.get('center', {})
            pixel_x = center.get('x')
            pixel_y = center.get('y')
            
            if pixel_x is None or pixel_y is None:
                return
            
            print(f"\n📍 检测到叶子 #{leaf.get('id', 0)}")
            
            # 第1步：像素 -> 3D相机坐标
            point_3d = self.pixel_to_3d(pixel_x, pixel_y)
            if point_3d is None:
                return
            
            # 第2步：相机坐标 -> 机械臂坐标
            pose_arm = self.transform_to_arm(point_3d)
            if pose_arm is None:
                return
            
            # 发布结果
            self.arm_pose_pub.publish(pose_arm)
            
            print("✓ 转换完成！")
            
        except Exception as e:
            self.get_logger().error(f'处理失败: {e}')


def main():
    """主函数"""
    rclpy.init()
    node = SimpleLeafToArmExample()
    
    print("\n" + "="*70)
    print("🌿 简单叶子到机械臂坐标转换示例")
    print("="*70)
    print("\n使用方法:")
    print("1. 启动相机和叶子检测")
    print("2. 启动机械臂驱动")
    print("3. 运行此脚本")
    print("4. 查看输出，了解转换过程")
    print("\n监控话题:")
    print("  - ros2 topic echo /leaf_detection/coordinates")
    print("  - ros2 topic echo /leaf_to_arm_result")
    print("\n按 Ctrl+C 停止")
    print("="*70 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✓ 节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

