import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class objectDetect(Node):

    def __init__(self):
        super().__init__('object_detect')
        
        # 检测相机类型
        self.camera_type = self.detect_camera_type()
        self.get_logger().info(f"检测到相机类型: {self.camera_type}")
        
        if self.camera_type == "realsense":
            # RealSense相机订阅
            self.image_sub = self.create_subscription( 
                Image, '/camera/camera/color/image_raw', self.arm_image_callback, 10)
            self.point_cloud_sub = self.create_subscription( 
                Image, '/camera/camera/aligned_depth_to_color/image_raw', self.arm_point_cloud_callback, 10)
            self.cam_info_sub = self.create_subscription( 
                CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.arm_image_depth_info_callback,10)
        else:
            # V4L2相机订阅（仅RGB，无深度）
            self.image_sub = self.create_subscription( 
                Image, '/image_raw', self.arm_image_callback, 10)
            self.point_cloud_sub = None
            self.cam_info_sub = None
            
        self.intrinsics = None
        self.depth_image = None

        # Timer definitions
        self.routine_timer = self.create_timer(0.05, self.routine_callback)

        # Transformation Interface
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # General Variables
        self.cv_image = None
        self.mask = None
        self.cv_bridge = CvBridge()

    def detect_camera_type(self):
        """检测可用的相机类型"""
        try:
            # 检查RealSense话题
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            if '/camera/camera/color/image_raw' in result.stdout:
                return "realsense"
            elif '/image_raw' in result.stdout:
                return "v4l2"
        except:
            pass
        return "unknown"

    def arm_image_depth_info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    # This gets bgr image from the image topic and finds where green in the image is
    def arm_image_callback(self, msg):      
        try:
            if self.camera_type == "realsense":
                self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                # V4L2相机可能需要不同的编码处理
                try:
                    self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                except CvBridgeError:
                    # 尝试其他编码
                    self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                    self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
            
        except Exception as e:
            self.get_logger().error(f"Error in arm_image_callback: {str(e)}")

    # This gets depth_frame aligned with RGB image
    def arm_point_cloud_callback(self, msg):
        try:
            if self.camera_type == "realsense":
                self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")

    def pixel_2_global(self, pixel_pt):
        if self.camera_type == "realsense" and self.depth_image is not None and self.intrinsics is not None:
            [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
            return [x, y,z]
        elif self.camera_type == "v4l2":
            # V4L2相机没有深度信息，返回固定深度
            # 这里可以根据实际应用调整
            return [0.0, 0.0, 0.5]  # 假设50cm深度
        else:
            return None

    def routine_callback(self):
        if (self.cv_image is None):
            return

        # TODO: COLOUR MASK TO FIND THE CENTER OF AN OBJECT OF INTEREST

        item_img_global = self.pixel_2_global([360, 240])

        if (item_img_global is None):
            return
        
        x = item_img_global[0] - 0.038 # OFFSET TO ACCOUNT FOR CAMERA OFFSET
        y = item_img_global[1]
        z = item_img_global[2] 
        
        self.get_logger().info(f"检测到物体位置 - x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "camera_color_optical_frame" if self.camera_type == "realsense" else "camera_link"
        transform_stamped.child_frame_id = "blue_object_frame"

        # TODO: COMPLETE TRANSFORMATION OUTPUT
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = z
        transform_stamped.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    rclpy.init()
    object_detect = objectDetect()
    rclpy.spin(object_detect)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



