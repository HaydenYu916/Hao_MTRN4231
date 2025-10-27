import rclpy
import cv2
import tf2_ros
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SimpleObjectDetect(Node):
    """简化的物体检测节点 - 兼容V4L2相机"""

    def __init__(self):
        super().__init__('simple_object_detect')
        
        # 订阅V4L2相机话题
        self.image_sub = self.create_subscription( 
            Image, '/image_raw', self.image_callback, 10)
        
        # Timer
        self.routine_timer = self.create_timer(0.1, self.routine_callback)

        # TF接口
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 变量
        self.cv_image = None
        self.cv_bridge = CvBridge()
        
        self.get_logger().info("简单物体检测节点已启动")

    def image_callback(self, msg):      
        try:
            # 尝试不同的编码方式
            try:
                self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError:
                try:
                    self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                    self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
                except CvBridgeError:
                    # 使用原始编码
                    self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                    if len(self.cv_image.shape) == 3:
                        self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_RGB2BGR)
            
        except Exception as e:
            self.get_logger().error(f"图像回调错误: {str(e)}")

    def routine_callback(self):
        if self.cv_image is None:
            return

        # 获取图像中心点
        height, width = self.cv_image.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        # 简单的颜色检测（检测绿色）
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
        # 绿色范围
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            
            # 计算质心
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # 在图像上标记
                cv2.circle(self.cv_image, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(self.cv_image, f"Object: ({cx}, {cy})", 
                           (cx-50, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                # 发布TF变换
                self.publish_object_transform(cx, cy)
                
                self.get_logger().info(f"检测到物体在像素位置: ({cx}, {cy})")
        
        # 显示图像（可选）
        cv2.imshow('Object Detection', self.cv_image)
        cv2.waitKey(1)

    def publish_object_transform(self, pixel_x, pixel_y):
        """发布物体位置的TF变换"""
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "camera_link"
        transform_stamped.child_frame_id = "detected_object"
        
        # 简单的像素到3D坐标转换（假设固定深度）
        # 这里可以根据实际相机标定进行调整
        depth = 0.5  # 假设50cm深度
        fx = 525.0   # 假设焦距
        fy = 525.0
        
        x = (pixel_x - 320) * depth / fx
        y = (pixel_y - 240) * depth / fy
        z = depth
        
        transform_stamped.transform.translation.x = x
        transform_stamped.transform.translation.y = y
        transform_stamped.transform.translation.z = z
        transform_stamped.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform_stamped)

def main():
    rclpy.init()
    node = SimpleObjectDetect()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



