#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from plantcv import plantcv as pcv
import json
import os
from datetime import datetime


class LeafDetector(Node):
    """
    叶子检测节点 - 使用PlantCV库
    订阅相机图像话题，检测图像中的叶子并发布检测结果
    发布标注的彩色图像（带边界框和标签）到 /leaf_detection/annotated_image
    """
    
    def __init__(self):
        super().__init__('leaf_detector')
        
        # 创建发布者和订阅者
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_publisher = self.create_publisher(
            String,
            '/leaf_detection_result',
            10
        )
        
        self.leaf_count_publisher = self.create_publisher(
            Int32,
            '/leaf_detection/leaf_count',
            10
        )
        
        self.coordinates_publisher = self.create_publisher(
            String,
            '/leaf_detection/coordinates',
            10
        )
        
        self.center_vector_publisher = self.create_publisher(
            Vector3,
            '/leaf_center_vector',
            10
        )
        
        # 添加边界框发布者
        self.bounding_box_publisher = self.create_publisher(
            MarkerArray,
            '/leaf_bounding_boxes',
            10
        )

        # 添加3D叶子标记发布者
        self.leaf_markers_publisher = self.create_publisher(
            MarkerArray,
            '/leaf_detection/leaf_markers',
            10
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # 添加标注图像发布者 (彩色标注，带边界框和标签)
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/leaf_detection/annotated_image',
            10
        )
        
        # 兼容旧系统的发布者 - 已移除
        # self.plantcv_annotated_publisher = self.create_publisher(
        #     Image,
        #     '/plantcv/image_annotated',
        #     10
        # )
        
        # 设置PlantCV参数
        pcv.params.debug = None  # 关闭调试输出
        
        self.get_logger().info('🌿 叶子检测节点已启动 (使用PlantCV)')
        self.frame_count = 0
    
    def image_callback(self, msg):
        """
        处理接收到的图像消息
        """
        try:
            self.frame_count += 1
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 使用PlantCV检测叶子
            detection_result, leaf_data, bounding_boxes = self.detect_leaves_with_plantcv(cv_image)
            
            # 发布检测结果
            result_msg = String()
            result_msg.data = detection_result
            self.detection_publisher.publish(result_msg)
            
            # 发布叶子数量
            count_msg = Int32()
            count_msg.data = leaf_data['num_leaves'] if leaf_data else 0
            self.leaf_count_publisher.publish(count_msg)
            
            # 发布坐标信息
            if leaf_data:
                coord_msg = String()
                coord_msg.data = json.dumps({
                    'frame': self.frame_count,
                    'timestamp': leaf_data.get('timestamp', ''),
                    'num_leaves': leaf_data['num_leaves'],
                    'coordinates': leaf_data.get('coordinates', [])
                })
                self.coordinates_publisher.publish(coord_msg)
            
            # 发布中心向量 (第一个叶子)
            if leaf_data and len(leaf_data.get('coordinates', [])) > 0:
                first_leaf = leaf_data['coordinates'][0]
                vector_msg = Vector3()
                vector_msg.x = float(first_leaf['center']['x'])
                vector_msg.y = float(first_leaf['center']['y'])
                vector_msg.z = float(first_leaf.get('area', 0))
                self.center_vector_publisher.publish(vector_msg)
            
            # 发布边界框
            if bounding_boxes is not None:
                self.publish_bounding_boxes(bounding_boxes, msg.header)
            
            # 发布标注的彩色图像 (带边界框和标签，如同index_colab.py)
            self.publish_annotated_image(cv_image, leaf_data, msg.header)
            
            # 发布3D叶子标记
            if leaf_data and len(leaf_data.get('coordinates', [])) > 0:
                self.publish_leaf_markers(leaf_data['coordinates'], msg.header)
            
            # 定期记录日志
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'✓ 帧 {self.frame_count}: {leaf_data["num_leaves"] if leaf_data else 0} 片叶子检测到')
            
        except Exception as e:
            self.get_logger().error(f'✗ 图像处理错误: {str(e)}')
            import traceback
            traceback.print_exc()
    
    def detect_leaves_with_plantcv(self, cv_image):
        """
        使用PlantCV库进行叶子检测
        返回: (检测结果字符串, 叶子数据字典, 边界框列表)
        """
        try:
            h, w = cv_image.shape[:2]
            
            # 最小裁剪
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            
            # HSV色彩空间检测绿色
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            lower_green = np.array([35, 50, 60])
            upper_green = np.array([85, 255, 255])
            thresh = cv2.inRange(hsv, lower_green, upper_green)
            thresh = (thresh / 255).astype(np.uint8)
            
            # 形态学处理
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # 轮廓检测
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                return "未检测到叶子", None, None
            
            # 面积过滤
            min_area = 500
            max_area = 50000
            min_circularity = 0.3
            
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area < min_area or area > max_area:
                    continue
                
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < min_circularity:
                    continue
                
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                if w_rect == 0 or h_rect == 0:
                    continue
                
                aspect_ratio = float(w_rect) / h_rect
                if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                    continue
                
                valid_contours.append(cnt)
            
            if len(valid_contours) == 0:
                return "未检测到有效的叶子", None, None
            
            # 提取坐标信息
            leaf_coordinates = []
            bounding_boxes = []
            
            for idx, cnt in enumerate(valid_contours, 1):
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # 计算中心点
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + 20  # 加上偏移量
                    cy = int(M['m01'] / M['m00']) + 20
                else:
                    cx = x + w_rect // 2 + 20
                    cy = y + h_rect // 2 + 20
                
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # 保存叶子信息
                leaf_info = {
                    'id': idx,
                    'center': {'x': cx, 'y': cy},
                    'bounding_box': {
                        'x': x + 20,
                        'y': y + 20,
                        'width': w_rect,
                        'height': h_rect,
                        'x_min': x + 20,
                        'y_min': y + 20,
                        'x_max': x + 20 + w_rect,
                        'y_max': y + 20 + h_rect
                    },
                    'area': float(area),
                    'perimeter': float(perimeter),
                }
                leaf_coordinates.append(leaf_info)
                
                # 边界框数据（用于RViz标记）
                bounding_boxes.append({
                    'id': idx,
                    'center_x': cx,
                    'center_y': cy,
                    'width': w_rect,
                    'height': h_rect,
                    'x': x + 20,
                    'y': y + 20
                })
            
            result = f"检测到 {len(valid_contours)} 片叶子"
            leaf_data = {
                'num_leaves': len(valid_contours),
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates
            }
            
            return result, leaf_data, bounding_boxes
            
        except Exception as e:
            self.get_logger().error(f'✗ PlantCV检测错误: {str(e)}')
            return f"检测错误: {str(e)}", None, None
    
    def publish_bounding_boxes(self, bounding_boxes, header):
        """
        发布边界框到RViz2
        """
        try:
            marker_array = MarkerArray()
            
            for bbox in bounding_boxes:
                marker = Marker()
                marker.header = header
                marker.header.frame_id = "camera_color_optical_frame"
                marker.id = bbox['id']
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                # 设置边界框位置和大小
                marker.pose.position.x = float(bbox['center_x']) / 1000.0
                marker.pose.position.y = float(bbox['center_y']) / 1000.0
                marker.pose.position.z = 0.0
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # 设置边界框大小
                marker.scale.x = float(bbox['width']) / 1000.0
                marker.scale.y = float(bbox['height']) / 1000.0
                marker.scale.z = 0.01
                
                # 设置颜色 (绿色)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                
                marker_array.markers.append(marker)
            
            self.bounding_box_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'✗ 边界框发布错误: {str(e)}')

    def publish_annotated_image(self, cv_image, leaf_data, header):
        """
        发布标注后的图像 (带边界框和标签，匹配index_colab.py风格)
        """
        try:
            annotated = cv_image.copy()
            
            if leaf_data is None or not leaf_data.get('coordinates'):
                # 如果没有检测到叶子，直接发布原图
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                annotated_msg.header = header
                self.annotated_image_publisher.publish(annotated_msg)
                return
            
            # 彩色列表（不同叶子使用不同颜色）
            colors = [
                (255, 0, 0),      # 蓝色
                (0, 255, 0),      # 绿色
                (0, 0, 255),      # 红色
                (255, 255, 0),    # 青色
                (255, 0, 255),    # 洋红色
                (0, 255, 255),    # 黄色
                (128, 0, 255),    # 紫色
                (255, 128, 0),    # 橙色
            ]
            
            # 为每个检测到的叶子绘制边界框和标签
            for leaf in leaf_data['coordinates']:
                obj_id = leaf['id']
                color = colors[(obj_id - 1) % len(colors)]
                
                # 获取边界框坐标
                bbox = leaf['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                w = bbox['width']
                h = bbox['height']
                
                # 绘制边界框
                cv2.rectangle(annotated, (x, y), (x_max, y_max), color, 2)
                
                # 绘制标签 (叶子ID)
                label = f'Leaf {obj_id}'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                # 添加背景矩形以提高文字可读性
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5), 
                            (x + label_size[0] + 5, label_y + 5), color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 绘制中心点
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                
                # 绘制圆形标记（可选）
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # 显示面积信息（可选）
                area_text = f"A:{leaf['area']:.0f}"
                cv2.putText(annotated, area_text, (x + 5, y_max - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 在左上角显示总数
            total_text = f"Leaves: {leaf_data['num_leaves']}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 发布标注图像
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            annotated_msg.header = header
            self.annotated_image_publisher.publish(annotated_msg)
            
            # 同时发布到旧话题以保持兼容性
            # self.plantcv_annotated_publisher.publish(annotated_msg) # 已移除
            
        except Exception as e:
            self.get_logger().error(f'✗ 标注图像发布错误: {str(e)}')

    def publish_leaf_markers(self, leaf_data, header):
        """
        发布3D叶子标记到RViz2
        """
        try:
            marker_array = MarkerArray()
            
            for leaf in leaf_data:
                marker = Marker()
                marker.header = header
                marker.header.frame_id = "camera_color_optical_frame"
                marker.id = leaf['id']
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # 设置中心点位置
                marker.pose.position.x = float(leaf['center']['x']) / 1000.0
                marker.pose.position.y = float(leaf['center']['y']) / 1000.0
                marker.pose.position.z = 0.0 # 假设叶子在Z轴上
                
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # 设置大小
                marker.scale.x = 0.05 # 半径
                marker.scale.y = 0.05 # 半径
                marker.scale.z = 0.05 # 半径
                
                # 设置颜色 (绿色)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                marker_array.markers.append(marker)
            
            self.leaf_markers_publisher.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'✗ 3D叶子标记发布错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    leaf_detector = LeafDetector()
    
    try:
        rclpy.spin(leaf_detector)
    except KeyboardInterrupt:
        print("\n✓ 叶子检测节点已停止")
    finally:
        leaf_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
