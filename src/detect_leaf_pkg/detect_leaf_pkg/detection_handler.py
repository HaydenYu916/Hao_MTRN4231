#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Detection Handler
Handles leaf detection using PlantCV library
"""

import cv2
import numpy as np
from plantcv import plantcv as pcv
from cv_bridge import CvBridge
from datetime import datetime
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image


class DetectionHandler:
    """Handles leaf detection logic using PlantCV"""
    
    def __init__(self, node, tf_handler=None):
        self.node = node
        self.tf_handler = tf_handler
        self.bridge = CvBridge()
        
        # PlantCV settings
        pcv.params.debug = None  # Disable debug output
        
        # Current frame data
        self.current_frame = None
        self.current_depth = None
        
        # Latest detection results (for service to use)
        self.latest_leaf_data = None
        self.latest_coordinates_base = None
        self.latest_leaf_attributes = {
            'has_yellow_tape': [],
            'yellow_ratio': [],
            'health_status': []
        }
        import threading
        self.latest_data_lock = threading.Lock()
        
        # Detection mode: continuous or on-demand
        self.continuous_detection = node.declare_parameter('continuous_detection', True).value
        self.min_area_default = 2000.0  # Default min area
        self.detect_yellow_tape = node.declare_parameter('detect_yellow_tape', True).value
        self.yellow_ratio_threshold = node.declare_parameter('yellow_ratio_threshold', 0.08).value  # 提高阈值到0.08以减少误检
        # HSV颜色范围参数（可配置，默认更严格以检测真正的黄色胶布）
        # 黄色胶布通常是亮黄色，饱和度和亮度都较高
        self.yellow_hsv_lower = node.declare_parameter('yellow_hsv_lower', [20, 100, 100]).value
        self.yellow_hsv_upper = node.declare_parameter('yellow_hsv_upper', [30, 255, 255]).value
        
        # 蓝色盒子检测参数
        self.detect_blue_box = node.declare_parameter('detect_blue_box', True).value
        self.blue_min_area = node.declare_parameter('blue_min_area', 3000.0).value
        # 蓝色在HSV中：H值在100-130之间（蓝色范围），S和V值较高
        self.blue_hsv_lower = node.declare_parameter('blue_hsv_lower', [100, 50, 50]).value
        self.blue_hsv_upper = node.declare_parameter('blue_hsv_upper', [130, 255, 255]).value
        
        # Publisher for detection results (for visualization node to subscribe)
        self.detection_results_pub = node.create_publisher(
            String,
            '/leaf_detection/detection_results',
            10
        )
        
        # Publisher for annotated image (continuously published)
        self.annotated_image_pub = node.create_publisher(
            Image,
            '/leaf_detection/annotated_image',
            10
        )
        
        self.healthy_leaves_pub = node.create_publisher(
            String,
            '/leaf_detection/healthy_leaves',
            10
        )
        self.unhealthy_leaves_pub = node.create_publisher(
            String,
            '/leaf_detection/unhealthy_leaves',
            10
        )
        
        # Publisher for blue box detection results
        self.blue_box_results_pub = node.create_publisher(
            String,
            '/leaf_detection/blue_box_results',
            10
        )
        
        # Publisher for MoveIt CollisionObjects (for dynamic_obstacles_monitor)
        from moveit_msgs.msg import CollisionObject
        self.moveit_collision_pub = node.create_publisher(
            CollisionObject,
            '/obsFromImg',
            10
        )
        
        self.get_logger().info('✓ DetectionHandler initialized')
        if self.continuous_detection:
            self.get_logger().info('📡 Continuous detection mode: images will be published continuously')
        if self.detect_yellow_tape:
            self.get_logger().info(
                f'🎯 Yellow tape detection enabled (threshold={self.yellow_ratio_threshold:.3f}, '
                f'HSV range: {self.yellow_hsv_lower} - {self.yellow_hsv_upper})'
            )
        if self.detect_blue_box:
            self.get_logger().info(
                f'📦 Blue box detection enabled (min_area={self.blue_min_area}, '
                f'HSV range: {self.blue_hsv_lower} - {self.blue_hsv_upper})'
            )
    
    def get_logger(self):
        """Get node logger"""
        return self.node.get_logger()
    
    def update_frames(self, color_msg, depth_msg):
        """Update current frame and depth from synchronized messages, and perform continuous detection"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            
            # Continuous detection mode: detect and publish on every frame
            if self.continuous_detection:
                self._perform_continuous_detection(color_msg.header)
        except Exception as e:
            self.get_logger().error(f'✗ Image conversion failed: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _perform_continuous_detection(self, header):
        """Perform detection on current frame and publish results continuously"""
        # Frame counter for debugging
        if not hasattr(self, '_frame_count'):
            self._frame_count = 0
        self._frame_count += 1
        
        if self.current_frame is None:
            self.get_logger().warn(f'Frame {self._frame_count}: current_frame is None')
            return
        
        if self.current_depth is None:
            self.get_logger().warn(f'Frame {self._frame_count}: current_depth is None, skipping detection')
            return
        
        try:
            # Perform detection with default parameters
            detection_result, leaf_data, bounding_boxes = self.detect_leaves_with_plantcv(
                self.current_frame,
                min_area=self.min_area_default
            )
            
            # Log detection status periodically
            if self._frame_count % 30 == 0:
                if leaf_data and leaf_data.get('num_leaves', 0) > 0:
                    self.get_logger().info(f'Frame {self._frame_count}: Detected {leaf_data["num_leaves"]} leaves, publishing image...')
                else:
                    self.get_logger().debug(f'Frame {self._frame_count}: No leaves detected, publishing original image...')
            
            # Update latest results (thread-safe)
            with self.latest_data_lock:
                if leaf_data and leaf_data.get('num_leaves', 0) > 0:
                    self.latest_leaf_data = leaf_data
                    # Also compute base coordinates for latest data
                    self.latest_coordinates_base = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
                    has_yellow, yellow_ratio, health_status = self._extract_leaf_attributes(
                        leaf_data.get('coordinates', [])
                    )
                    self.latest_leaf_attributes = {
                        'has_yellow_tape': has_yellow,
                        'yellow_ratio': yellow_ratio,
                        'health_status': health_status
                    }
                    
                    # Log base frame coordinates periodically
                    if self._frame_count % 30 == 0 and self.latest_coordinates_base:
                        self.get_logger().info('📍 Latest base coordinates:')
                        for i, point in enumerate(self.latest_coordinates_base):
                            self.get_logger().info(f'  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m')
                else:
                    self.latest_leaf_data = None
                    self.latest_coordinates_base = []
                    self.latest_leaf_attributes = {
                        'has_yellow_tape': [],
                        'yellow_ratio': [],
                        'health_status': []
                    }
            
            # Publish health status summary
            self._publish_health_status(leaf_data)
            
            # Publish detection results to topic (for visualization node)
            if leaf_data:
                # Convert to base coordinates if tf_handler is available
                base_coords_list = []
                if self.tf_handler:
                    base_coords_msg = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
                    # Convert Point messages to dict for JSON serialization
                    for point_msg in base_coords_msg:
                        base_coords_list.append({
                            'x': point_msg.x,
                            'y': point_msg.y,
                            'z': point_msg.z
                        })
                
                # Clean blue boxes for JSON serialization (remove numpy arrays)
                blue_boxes_clean = self._clean_blue_boxes_for_json(leaf_data.get('blue_boxes', []))
                
                results_json = {
                    'num_leaves': leaf_data['num_leaves'],
                    'timestamp': leaf_data.get('timestamp', ''),
                    'coordinates': leaf_data.get('coordinates', []),  # Camera frame coordinates
                    'base_coordinates': base_coords_list if base_coords_list else [],  # Base frame coordinates
                    'blue_boxes': blue_boxes_clean  # Blue box coordinates (cleaned for JSON)
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
                
                # Publish blue box results separately
                blue_boxes = leaf_data.get('blue_boxes', [])
                if len(blue_boxes) > 0:
                    blue_box_json = {
                        'num_blue_boxes': len(blue_boxes),
                        'timestamp': leaf_data.get('timestamp', ''),
                        'blue_boxes': blue_boxes_clean
                    }
                    blue_box_msg = String()
                    blue_box_msg.data = json.dumps(blue_box_json)
                    self.blue_box_results_pub.publish(blue_box_msg)
                    
                    # Publish blue boxes as MoveIt CollisionObjects to /obsFromImg
                    self._publish_blue_boxes_to_moveit(blue_boxes)
            else:
                # Publish empty results if no leaves detected
                results_json = {
                    'num_leaves': 0,
                    'timestamp': datetime.now().isoformat(),
                    'coordinates': [],
                    'blue_boxes': []
                }
                results_msg = String()
                results_msg.data = json.dumps(results_json)
                self.detection_results_pub.publish(results_msg)
            
            # Publish annotated image (with or without detections)
            annotated_image = self.draw_annotations(
                self.current_frame, 
                leaf_data.get('coordinates', []) if leaf_data else [],
                leaf_data.get('blue_boxes', []) if leaf_data else []
            )
            if annotated_image is not None:
                try:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    annotated_msg.header = header
                    self.annotated_image_pub.publish(annotated_msg)
                except Exception as e:
                    self.get_logger().error(f'✗ Error publishing annotated image: {str(e)}')
                    import traceback
                    self.get_logger().error(traceback.format_exc())
                    
        except Exception as e:
            self.get_logger().error(f'✗ Continuous detection error: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            # Still publish original image even if detection fails
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8")
                annotated_msg.header = header
                self.annotated_image_pub.publish(annotated_msg)
            except:
                pass
    
    def _publish_health_status(self, leaf_data):
        """Publish healthy/unhealthy leaf summaries as separate topics"""
        healthy_leaves = []
        unhealthy_leaves = []
        
        if leaf_data and leaf_data.get('coordinates'):
            for leaf in leaf_data['coordinates']:
                is_unhealthy = bool(leaf.get('has_yellow_tape', False))
                payload = {
                    'id': leaf.get('id'),
                    'center': leaf.get('center', {}),
                    'bounding_box': leaf.get('bounding_box', {}),
                    'yellow_ratio': leaf.get('yellow_ratio', 0.0),
                    'health_status': 'unhealthy' if is_unhealthy else 'healthy'
                }
                if is_unhealthy:
                    unhealthy_leaves.append(payload)
                else:
                    healthy_leaves.append(payload)
        else:
            # No leaves detected, publish empty payloads
            healthy_leaves = []
            unhealthy_leaves = []
        
        healthy_msg = String()
        healthy_msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'num_leaves': len(healthy_leaves),
            'leaves': healthy_leaves
        })
        unhealthy_msg = String()
        unhealthy_msg.data = json.dumps({
            'timestamp': datetime.now().isoformat(),
            'num_leaves': len(unhealthy_leaves),
            'leaves': unhealthy_leaves
        })
        
        self.healthy_leaves_pub.publish(healthy_msg)
        self.unhealthy_leaves_pub.publish(unhealthy_msg)
    
    def _convert_to_base_coordinates(self, leaf_coordinates):
        """Convert leaf coordinates to base frame"""
        coordinates = []
        from geometry_msgs.msg import Point
        
        if self.tf_handler:
            for i, leaf in enumerate(leaf_coordinates):
                point_3d = leaf.get('point_3d')
                if point_3d is not None:
                    # Log original camera optical frame coordinates
                    self.get_logger().debug(
                        f'Leaf {i+1} - Camera optical frame: '
                        f'({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})'
                    )
                    
                    base_coords = self.tf_handler.camera_to_base(point_3d)
                    if base_coords:
                        self.get_logger().debug(
                            f'Leaf {i+1} - Base_link frame (after TF + offsets): '
                            f'({base_coords[0]:.3f}, {base_coords[1]:.3f}, {base_coords[2]:.3f})'
                        )
                        
                        point_msg = Point()
                        point_msg.x = float(base_coords[0])
                        point_msg.y = float(base_coords[1])
                        point_msg.z = float(base_coords[2])
                        coordinates.append(point_msg)
                    else:
                        self.get_logger().warn(f'Leaf {i+1} - Failed to convert to base coordinates')
        else:
            for leaf in leaf_coordinates:
                point_3d = leaf.get('point_3d')
                if point_3d is not None:
                    point_msg = Point()
                    point_msg.x = float(point_3d[0])
                    point_msg.y = float(point_3d[1])
                    point_msg.z = float(point_3d[2])
                    coordinates.append(point_msg)
        
        return coordinates
    
    def _extract_leaf_attributes(self, leaf_records):
        """Extract yellow tape indicators and health labels from detection results"""
        has_yellow = []
        yellow_ratio = []
        health_status = []
        for leaf in leaf_records or []:
            has_yellow.append(bool(leaf.get('has_yellow_tape', False)))
            yellow_ratio.append(float(leaf.get('yellow_ratio', 0.0)))
            health_status.append(str(leaf.get('health_status', 'unknown')))
        return has_yellow, yellow_ratio, health_status
    
    def _clean_blue_boxes_for_json(self, blue_boxes):
        """Clean blue box data for JSON serialization (remove numpy arrays)"""
        cleaned = []
        for box in blue_boxes:
            cleaned_box = {
                'id': box.get('id'),
                'type': box.get('type'),
                'num_faces': box.get('num_faces', 1),
                'center': box.get('center', {}),
                'bounding_box': box.get('bounding_box', {}),
                'area': box.get('area', 0.0),
                'depth_mm': box.get('depth_mm', 0.0),
                'point_3d': list(box.get('point_3d')) if box.get('point_3d') is not None else None,
                'size_3d': box.get('size_3d')
            }
            # Remove 'faces' field as it contains numpy arrays (contours)
            cleaned.append(cleaned_box)
        return cleaned
    
    def _publish_blue_boxes_to_moveit(self, blue_boxes):
        """Publish blue boxes as MoveIt CollisionObjects to /obsFromImg"""
        from moveit_msgs.msg import CollisionObject
        from std_msgs.msg import Header
        
        for box in blue_boxes:
            point_3d = box.get('point_3d')
            size_3d = box.get('size_3d')
            
            if point_3d is None or size_3d is None:
                continue
            
            # Convert camera frame coordinates to base frame if tf_handler is available
            base_point_3d = point_3d
            if self.tf_handler:
                base_coords = self.tf_handler.camera_to_base(point_3d)
                if base_coords:
                    base_point_3d = base_coords
                else:
                    self.get_logger().warn(f'Failed to convert blue box {box.get("id")} to base frame')
                    continue
            
            # Create CollisionObject message
            collision_obj = CollisionObject()
            collision_obj.header = Header()
            collision_obj.header.frame_id = "base_link"  # Use base_link frame for MoveIt
            collision_obj.header.stamp = self.node.get_clock().now().to_msg()
            
            collision_obj.id = f"blue_box_{box['id']:03d}"
            collision_obj.operation = CollisionObject.ADD  # 0 = ADD
            
            # Set primitive (box)
            from shape_msgs.msg import SolidPrimitive
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX  # 1 = BOX
            primitive.dimensions = [
                float(size_3d['width']),
                float(size_3d['height']),
                float(size_3d['depth'])
            ]
            collision_obj.primitives = [primitive]
            
            # Set pose
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = float(base_point_3d[0])
            pose.position.y = float(base_point_3d[1])
            pose.position.z = float(base_point_3d[2])
            pose.orientation.w = 1.0  # No rotation
            collision_obj.primitive_poses = [pose]
            
            # Publish
            self.moveit_collision_pub.publish(collision_obj)
            self.get_logger().debug(
                f'Published blue box {collision_obj.id} to /obsFromImg: '
                f'pos=({base_point_3d[0]:.3f}, {base_point_3d[1]:.3f}, {base_point_3d[2]:.3f}), '
                f'size=({size_3d["width"]:.3f}, {size_3d["height"]:.3f}, {size_3d["depth"]:.3f})'
            )
    
    async def handle_request(self, request):
        """Handle detection service request"""
        if request.command == "detect":
            return await self._detect_leaves(request)
        else:
            return {
                'success': False,
                'message': f"Unknown command: {request.command}",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': ''
            }
    
    async def _detect_leaves(self, request):
        """Handle detection service request - use latest results or re-detect with custom params"""
        if self.current_frame is None or self.current_depth is None:
            return {
                'success': False,
                'message': "No frame available",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': ''
            }
        
        try:
            # If min_area is 0 or matches default, use latest continuous detection results
            use_custom_params = (request.min_area > 0 and request.min_area != self.min_area_default)
            
            if not use_custom_params and self.continuous_detection:
                # Use latest detection results (faster, no re-computation)
                with self.latest_data_lock:
                    if self.latest_coordinates_base is not None:
                        num_leaves = len(self.latest_coordinates_base)
                        if num_leaves > 0:
                            attrs = self.latest_leaf_attributes
                            return {
                                'success': True,
                                'message': f"Detected {num_leaves} leaves (using latest continuous detection)",
                                'coordinates': self.latest_coordinates_base.copy(),
                                'num_leaves': num_leaves,
                                'has_yellow_tape': list(attrs.get('has_yellow_tape', [])),
                                'yellow_ratio': list(attrs.get('yellow_ratio', [])),
                                'health_status': list(attrs.get('health_status', [])),
                                'debug_info': json.dumps({
                                    'source': 'latest_continuous_detection',
                                    'has_base_coords': self.tf_handler is not None
                                })
                            }
                        else:
                            return {
                                'success': False,
                                'message': "No leaves detected in latest frame",
                                'coordinates': [],
                                'num_leaves': 0,
                                'has_yellow_tape': [],
                                'yellow_ratio': [],
                                'health_status': [],
                                'debug_info': json.dumps({'source': 'latest_continuous_detection'})
                            }
            
            # Re-detect with custom parameters or if continuous detection is disabled
            min_area = request.min_area if request.min_area > 0 else self.min_area_default
            detection_result, leaf_data, bounding_boxes = self.detect_leaves_with_plantcv(
                self.current_frame,
                min_area=min_area
            )
            
            if leaf_data is None or leaf_data.get('num_leaves', 0) == 0:
                return {
                    'success': False,
                    'message': detection_result,
                    'coordinates': [],
                    'num_leaves': 0,
                    'has_yellow_tape': [],
                    'yellow_ratio': [],
                    'health_status': [],
                    'debug_info': json.dumps({'detection_result': detection_result, 'min_area': min_area})
                }
            
            # Convert to base frame coordinates
            coordinates = self._convert_to_base_coordinates(leaf_data.get('coordinates', []))
            has_yellow, yellow_ratio, health_status = self._extract_leaf_attributes(
                leaf_data.get('coordinates', [])
            )
            
            return {
                'success': True,
                'message': f"Detected {len(coordinates)} leaves",
                'coordinates': coordinates,
                'num_leaves': len(coordinates),
                'has_yellow_tape': has_yellow,
                'yellow_ratio': yellow_ratio,
                'health_status': health_status,
                'debug_info': json.dumps({
                    'num_leaves': leaf_data['num_leaves'],
                    'timestamp': leaf_data.get('timestamp', ''),
                    'has_base_coords': self.tf_handler is not None,
                    'min_area': min_area,
                    'source': 'on_demand_detection'
                })
            }
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            import traceback
            return {
                'success': False,
                'message': f"Detection failed: {str(e)}",
                'coordinates': [],
                'num_leaves': 0,
                'has_yellow_tape': [],
                'yellow_ratio': [],
                'health_status': [],
                'debug_info': json.dumps({'error': str(e), 'traceback': traceback.format_exc()})
            }
    
    def detect_leaves_with_plantcv(self, cv_image, min_area=None):
        """
        Detect leaves using PlantCV library
        Returns: (detection_result_string, leaf_data_dict, bounding_boxes_list)
        """
        try:
            h, w = cv_image.shape[:2]
            
            # Minimal cropping
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            
            # HSV color space - detect green and optional yellow tape
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh_green = cv2.inRange(hsv, lower_green, upper_green)
            thresh_green = (thresh_green / 255).astype(np.uint8)
            
            # 步骤2.5: 检测蓝色盒子（独立于叶子检测）
            blue_box_coordinates = []
            if self.detect_blue_box:
                # 使用HSV颜色空间检测蓝色
                lower_blue_hsv = np.array(self.blue_hsv_lower, dtype=np.uint8)
                upper_blue_hsv = np.array(self.blue_hsv_upper, dtype=np.uint8)
                thresh_blue_hsv = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
                
                # 形态学处理：去除小噪声，连接相近区域
                kernel_blue_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                kernel_blue_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
                
                # 先闭运算连接相近区域，再开运算去除小噪声
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_hsv, cv2.MORPH_CLOSE, kernel_blue_medium, iterations=2)
                thresh_blue_morph = cv2.morphologyEx(thresh_blue_morph, cv2.MORPH_OPEN, kernel_blue_small, iterations=1)
                
                # 检测蓝色盒子的轮廓（包括所有层级，以检测多个面）
                blue_contours, blue_hierarchy = cv2.findContours(thresh_blue_morph, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # 收集所有有效的蓝色区域（可能是盒子的不同面）
                valid_blue_regions = []
                
                # 过滤蓝色盒子轮廓（检测所有可见面）
                for idx, blue_cnt in enumerate(blue_contours):
                    area = cv2.contourArea(blue_cnt)
                    
                    # 降低单个面的最小面积要求，因为我们要检测多个面
                    min_face_area = max(500, self.blue_min_area * 0.3)  # 单个面至少是总面积的30%或500像素
                    if area < min_face_area:
                        continue
                    
                    # 计算边界框
                    x, y, w_rect, h_rect = cv2.boundingRect(blue_cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                    
                    # 计算矩形度（轮廓面积与边界框面积的比值）
                    bbox_area = w_rect * h_rect
                    if bbox_area == 0:
                        continue
                    rect_ratio = area / bbox_area
                    
                    # 计算宽高比
                    aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                    
                    # 更宽松的过滤条件（因为可能是盒子的侧面，形状可能不完全规则）
                    if rect_ratio < 0.4:  # 降低到0.4
                        continue
                    
                    if aspect_ratio < 0.2 or aspect_ratio > 5.0:  # 放宽范围
                        continue
                    
                    # 计算中心点
                    M = cv2.moments(blue_cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                    else:
                        cx = x + w_rect // 2 + 20
                        cy = y + h_rect // 2 + 20
                    
                    # 获取深度值和3D坐标（采样多个点以获得更准确的深度）
                    depth_value_mm = 0
                    point_3d = None
                    
                    if self.current_depth is not None and self.tf_handler:
                        try:
                            # 在轮廓区域内采样多个点
                            mask = np.zeros((crop_img.shape[0], crop_img.shape[1]), dtype=np.uint8)
                            cv2.drawContours(mask, [blue_cnt], -1, 255, -1)
                            
                            # 转换到原始图像坐标（深度图像是原始尺寸）
                            depth_x = x + 20
                            depth_y = y + 20
                            depth_x_end = min(self.current_depth.shape[1], depth_x + w_rect)
                            depth_y_end = min(self.current_depth.shape[0], depth_y + h_rect)
                            depth_x_start = max(0, depth_x)
                            depth_y_start = max(0, depth_y)
                            
                            # 在掩码区域内采样深度值
                            mask_roi = mask[y:y+h_rect, x:x+w_rect]
                            depth_roi = self.current_depth[depth_y_start:depth_y_end, depth_x_start:depth_x_end]
                            
                            # 调整mask ROI的大小以匹配depth ROI
                            if mask_roi.shape != depth_roi.shape:
                                mask_roi_resized = cv2.resize(mask_roi, (depth_roi.shape[1], depth_roi.shape[0]))
                            else:
                                mask_roi_resized = mask_roi
                            
                            valid_depths = depth_roi[(mask_roi_resized > 0) & (depth_roi > 0)]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.median(valid_depths))  # 使用中位数更稳定
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                        except Exception as e:
                            self.get_logger().debug(f'蓝色区域深度获取错误: {e}')
                            pass
                    
                    # 保存蓝色区域信息（可能是盒子的一个面）
                    region_info = {
                        'contour': blue_cnt,
                        'area': float(area),
                        'bbox': (x, y, w_rect, h_rect),
                        'center_2d': (cx, cy),
                        'depth_mm': float(depth_value_mm),
                        'point_3d': point_3d,
                        'rect_ratio': float(rect_ratio),
                        'aspect_ratio': float(aspect_ratio),
                    }
                    valid_blue_regions.append(region_info)
                
                # 根据深度和位置将区域分组为同一个盒子
                blue_box_groups = []
                depth_tolerance = 50  # 深度容差（毫米）
                position_tolerance = 100  # 位置容差（像素）
                
                for region in valid_blue_regions:
                    if region['point_3d'] is None:
                        # 如果没有深度信息，每个区域单独成组
                        blue_box_groups.append([region])
                        continue
                    
                    # 尝试找到匹配的组
                    matched = False
                    for group in blue_box_groups:
                        # 检查是否有深度信息
                        group_depths = [r['depth_mm'] for r in group if r['point_3d'] is not None]
                        if len(group_depths) == 0:
                            continue
                        
                        avg_group_depth = np.mean(group_depths)
                        depth_diff = abs(region['depth_mm'] - avg_group_depth)
                        
                        # 检查位置是否相近
                        group_centers = [r['center_2d'] for r in group]
                        avg_center = (np.mean([c[0] for c in group_centers]), 
                                     np.mean([c[1] for c in group_centers]))
                        pos_diff = np.sqrt((region['center_2d'][0] - avg_center[0])**2 + 
                                          (region['center_2d'][1] - avg_center[1])**2)
                        
                        # 如果深度和位置都相近，加入该组
                        if depth_diff < depth_tolerance and pos_diff < position_tolerance:
                            group.append(region)
                            matched = True
                            break
                    
                    if not matched:
                        # 创建新组
                        blue_box_groups.append([region])
                
                # 为每个组创建一个完整的蓝色盒子
                blue_box_idx = 1
                for group in blue_box_groups:
                    if len(group) == 0:
                        continue
                    
                    # 计算合并后的边界框（包含所有面）
                    all_x_min = min([r['bbox'][0] for r in group])
                    all_y_min = min([r['bbox'][1] for r in group])
                    all_x_max = max([r['bbox'][0] + r['bbox'][2] for r in group])
                    all_y_max = max([r['bbox'][1] + r['bbox'][3] for r in group])
                    
                    merged_w = all_x_max - all_x_min
                    merged_h = all_y_max - all_y_min
                    merged_area = sum([r['area'] for r in group])
                    
                    # 计算合并后的中心点
                    merged_cx = (all_x_min + all_x_max) // 2 + 20
                    merged_cy = (all_y_min + all_y_max) // 2 + 20
                    
                    # 计算平均深度和3D坐标
                    valid_depths = [r['depth_mm'] for r in group if r['depth_mm'] > 0]
                    avg_depth_mm = int(np.mean(valid_depths)) if len(valid_depths) > 0 else 0
                    
                    # 获取3D坐标
                    merged_point_3d = None
                    if avg_depth_mm > 0 and self.current_depth is not None and self.tf_handler:
                        try:
                            merged_point_3d = self.tf_handler.pixel_to_3d(merged_cx, merged_cy, avg_depth_mm)
                        except:
                            pass
                    
                    # 计算盒子的3D尺寸（使用深度信息）
                    box_3d_size = None
                    if len(valid_depths) > 0 and merged_point_3d and self.tf_handler and self.tf_handler.intrinsics:
                        # 估算3D尺寸（基于像素尺寸和深度）
                        depth_m = avg_depth_mm * 0.001
                        pixel_size = depth_m / self.tf_handler.intrinsics.fx if self.tf_handler.intrinsics.fx > 0 else 0.001
                        box_3d_size = {
                            'width': merged_w * pixel_size,
                            'height': merged_h * pixel_size,
                            'depth': (max(valid_depths) - min(valid_depths)) * 0.001 if len(valid_depths) > 1 else 0.1
                        }
                    
                    # 保存完整的蓝色盒子信息
                    blue_box_info = {
                        'id': blue_box_idx,
                        'type': 'blue_box',
                        'num_faces': len(group),  # 检测到的面数
                        'center': {'x': merged_cx, 'y': merged_cy},
                        'bounding_box': {
                            'x': all_x_min + 20,
                            'y': all_y_min + 20,
                            'width': merged_w,
                            'height': merged_h,
                            'x_min': all_x_min + 20,
                            'y_min': all_y_min + 20,
                            'x_max': all_x_max + 20,
                            'y_max': all_y_max + 20
                        },
                        'area': float(merged_area),
                        'depth_mm': float(avg_depth_mm),
                        'point_3d': merged_point_3d,
                        'size_3d': box_3d_size,
                        'faces': group  # 保存所有面的信息
                    }
                    blue_box_coordinates.append(blue_box_info)
                    
                    blue_box_idx += 1
                
                # 调试信息
                if len(blue_box_coordinates) > 0:
                    self.get_logger().info(f'📦 检测到 {len(blue_box_coordinates)} 个蓝色盒子')
            
            thresh_yellow_full = None
            if self.detect_yellow_tape:
                # 使用可配置的HSV范围，默认范围更严格以检测真正的黄色胶布
                lower_yellow = np.array(self.yellow_hsv_lower, dtype=np.uint8)
                upper_yellow = np.array(self.yellow_hsv_upper, dtype=np.uint8)
                thresh_yellow_hsv = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # 使用更严格的形态学操作，去除小的噪声点
                # 先进行开运算去除小噪声，然后进行闭运算连接相近区域
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_OPEN, kernel_small, iterations=1)
                thresh_yellow_hsv = cv2.morphologyEx(thresh_yellow_hsv, cv2.MORPH_CLOSE, kernel_small, iterations=1)
                
                # 使用LAB颜色空间作为补充，但使用更严格的范围
                # 只检测真正的亮黄色胶布，避免检测到叶子本身的黄色部分
                lab = cv2.cvtColor(crop_img, cv2.COLOR_BGR2LAB)
                # 黄色胶布通常是亮黄色：L较高（亮度高），a较低（偏绿），b很高（偏黄）
                # 使用更严格的范围，避免误检叶子边缘的黄色
                lab_yellow_mask = np.zeros_like(thresh_yellow_hsv, dtype=np.uint8)
                # 更严格的条件：L>110（较亮），a<140（偏绿），b>150（很黄）
                lab_yellow_mask[(lab[:,:,0] > 110) & (lab[:,:,1] < 140) & (lab[:,:,2] > 150)] = 255
                
                # 合并HSV和LAB的检测结果（使用AND操作，两个都检测到才认为是黄色胶布）
                # 这样可以减少误检
                thresh_yellow_full = cv2.bitwise_and(thresh_yellow_hsv, lab_yellow_mask)
                
                # 如果AND结果太少，则使用OR作为备选（但需要更高的阈值）
                yellow_pixels_and = np.sum(thresh_yellow_full > 0)
                yellow_pixels_hsv = np.sum(thresh_yellow_hsv > 0)
                if yellow_pixels_and < yellow_pixels_hsv * 0.3:  # 如果AND结果太少，使用OR
                    thresh_yellow_full = cv2.bitwise_or(thresh_yellow_hsv, lab_yellow_mask)
                    # 但需要额外的形态学处理去除小区域
                    kernel_medium = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                    thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_OPEN, kernel_medium, iterations=1)
            
            # Morphological processing
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh_green = cv2.morphologyEx(thresh_green, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh_green = cv2.morphologyEx(thresh_green, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # Contour detection
            contours, _ = cv2.findContours(thresh_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                return "No leaves detected", None, None
            
            # Filtering parameters
            min_area_threshold = min_area if min_area and min_area > 0 else 2000
            max_area = 50000
            min_circularity = 0.2
            max_circularity = 0.9
            
            valid_contours = []
            for cnt in contours:
                area = cv2.contourArea(cnt)
                
                if area < min_area_threshold or area > max_area:
                    continue
                
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if circularity < min_circularity or circularity > max_circularity:
                    continue
                
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                if w_rect == 0 or h_rect == 0:
                    continue
                
                aspect_ratio = float(w_rect) / h_rect if h_rect > 0 else 0
                if aspect_ratio < 0.4 or aspect_ratio > 2.5:
                    continue
                
                min_dimension = min(w_rect, h_rect)
                if min_dimension < 30:
                    continue
                
                valid_contours.append(cnt)
            
            if len(valid_contours) == 0:
                return "No valid leaves detected", None, None
            
            # Extract coordinates (with depth filtering)
            leaf_coordinates = []
            bounding_boxes = []
            leaf_idx = 1
            
            for cnt in valid_contours:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # Calculate center point
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + 20
                    cy = int(M['m01'] / M['m00']) + 20
                else:
                    cx = x + w_rect // 2 + 20
                    cy = y + h_rect // 2 + 20
                
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # Get depth value and 3D coordinates
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if self.current_depth is not None and self.tf_handler:
                    try:
                        if cx < self.current_depth.shape[1] and cy < self.current_depth.shape[0]:
                            depth_value_mm = int(self.current_depth[cy, cx])
                            
                            if depth_value_mm > 0 and 100 <= depth_value_mm <= 2000:
                                has_valid_depth = True
                                point_3d = self.tf_handler.pixel_to_3d(cx, cy, depth_value_mm)
                    except:
                        pass
                
                if not has_valid_depth:
                    continue
                
                has_yellow_tape = False
                yellow_ratio = 0.0
                if self.detect_yellow_tape and thresh_yellow_full is not None:
                    # 创建叶子区域的掩码（使用完整的轮廓，而不是边界框）
                    # 这样可以更准确地检测叶子内部的黄色区域
                    leaf_mask = np.zeros_like(thresh_green, dtype=np.uint8)
                    cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                    
                    # 直接在完整掩码上计算，而不是使用边界框区域
                    # 这样可以避免边界框可能包含非叶子区域的问题
                    leaf_pixels = np.sum(leaf_mask > 0)
                    yellow_pixels = np.sum((leaf_mask > 0) & (thresh_yellow_full > 0))
                    
                    if leaf_pixels > 0:
                        yellow_ratio = yellow_pixels / leaf_pixels
                        
                        # 额外的验证：检查黄色区域的连通性和大小
                        # 真正的黄色胶布应该是连续的、有一定大小的区域
                        yellow_in_leaf = (leaf_mask > 0) & (thresh_yellow_full > 0)
                        yellow_contours, _ = cv2.findContours(
                            yellow_in_leaf.astype(np.uint8), 
                            cv2.RETR_EXTERNAL, 
                            cv2.CHAIN_APPROX_SIMPLE
                        )
                        
                        # 计算最大黄色连通区域的面积
                        max_yellow_area = 0
                        if len(yellow_contours) > 0:
                            max_yellow_area = max(cv2.contourArea(c) for c in yellow_contours)
                        
                        # 黄色区域应该至少占叶子面积的1%，且最大连通区域应该足够大（至少100像素）
                        min_yellow_area_threshold = max(100, leaf_pixels * 0.01)
                        
                        # 更严格的判断：既要满足比例阈值，又要满足最小区域要求
                        if (yellow_ratio >= self.yellow_ratio_threshold and 
                            max_yellow_area >= min_yellow_area_threshold):
                            has_yellow_tape = True
                            
                            # 调试日志：记录检测到的黄色标签信息
                            self.get_logger().info(
                                f'🎯 Leaf {leaf_idx}: 检测到黄色标签 - '
                                f'yellow_ratio={yellow_ratio:.4f} '
                                f'(阈值={self.yellow_ratio_threshold:.4f}), '
                                f'黄色像素={yellow_pixels}/{leaf_pixels}, '
                                f'最大连通区域={max_yellow_area:.0f}像素'
                            )
                        else:
                            # 调试日志：记录未通过验证的情况
                            if yellow_ratio > 0.01:  # 只记录有明显黄色但未达阈值的情况
                                reason = []
                                if yellow_ratio < self.yellow_ratio_threshold:
                                    reason.append(f'比例不足({yellow_ratio:.4f}<{self.yellow_ratio_threshold:.4f})')
                                if max_yellow_area < min_yellow_area_threshold:
                                    reason.append(f'区域太小({max_yellow_area:.0f}<{min_yellow_area_threshold:.0f})')
                                self.get_logger().debug(
                                    f'⚠️ Leaf {leaf_idx}: 检测到黄色但未通过验证 - '
                                    f'yellow_ratio={yellow_ratio:.4f}, '
                                    f'最大连通区域={max_yellow_area:.0f}像素, '
                                    f'原因: {", ".join(reason)}'
                                )
                    else:
                        # 调试：如果叶子区域为空，记录警告
                        self.get_logger().warn(
                            f'⚠️ Leaf {leaf_idx}: 叶子区域掩码为空，无法检测黄色标签 '
                            f'(bbox: x={x}, y={y}, w={w_rect}, h={h_rect}, '
                            f'mask_shape={leaf_mask.shape})'
                        )
                
                # Save leaf info
                leaf_info = {
                    'id': leaf_idx,
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
                    'depth_mm': float(depth_value_mm),
                    'point_3d': point_3d,
                    'has_yellow_tape': has_yellow_tape,
                    'yellow_ratio': float(yellow_ratio),
                    'health_status': 'unhealthy' if has_yellow_tape else 'healthy'
                }
                leaf_coordinates.append(leaf_info)
                
                bounding_boxes.append({
                    'id': leaf_idx,
                    'center_x': cx,
                    'center_y': cy,
                    'width': w_rect,
                    'height': h_rect,
                    'x': x + 20,
                    'y': y + 20
                })
                
                leaf_idx += 1
            
            
            num_detected_leaves = len(leaf_coordinates)
            
            if num_detected_leaves == 0:
                return "No valid leaves detected (after depth filtering)", None, None
            
            # 统计黄色标签检测结果
            yellow_tape_count = sum(1 for leaf in leaf_coordinates if leaf.get('has_yellow_tape', False))
            if yellow_tape_count > 0:
                self.get_logger().info(
                    f'📊 检测总结: 共{num_detected_leaves}片叶子, '
                    f'其中{yellow_tape_count}片检测到黄色标签'
                )
                # 列出所有有黄色标签的叶子
                for leaf in leaf_coordinates:
                    if leaf.get('has_yellow_tape', False):
                        self.get_logger().info(
                            f'  ✓ Leaf {leaf["id"]}: yellow_ratio={leaf.get("yellow_ratio", 0):.4f}'
                        )
            
            result = f"Detected {num_detected_leaves} leaves"
            if len(blue_box_coordinates) > 0:
                result += f", {len(blue_box_coordinates)} blue boxes"
            
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'num_blue_boxes': len(blue_box_coordinates),
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates,
                'blue_boxes': blue_box_coordinates
            }
            
            return result, leaf_data, bounding_boxes
            
        except Exception as e:
            self.get_logger().error(f'✗ PlantCV detection error: {str(e)}')
            return f"Detection error: {str(e)}", None, None
    
    def draw_annotations(self, display_frame, leaf_coordinates, blue_box_coordinates=None):
        """Draw annotations on image"""
        if display_frame is None:
            return None
        
        # Always return a copy of the frame (with or without annotations)
        annotated = display_frame.copy()
        
        if blue_box_coordinates is None:
            blue_box_coordinates = []
        
        if not leaf_coordinates and not blue_box_coordinates:
            # No leaves or boxes detected, just return original frame
            return annotated
        
        # Draw annotations for detected leaves
        try:
            colors = [
                (255, 0, 0),      # Blue
                (0, 255, 0),      # Green
                (0, 0, 255),      # Red
                (255, 255, 0),    # Cyan
                (255, 0, 255),    # Magenta
                (0, 255, 255),    # Yellow
            ]
            
            for leaf in leaf_coordinates:
                obj_id = leaf['id']
                color = colors[(obj_id - 1) % len(colors)]
                
                bbox = leaf['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # Draw bounding box
                cv2.rectangle(annotated, (x, y), (x_max, y_max), color, 2)
                
                # Draw label
                label = f'Leaf {obj_id}'
                if leaf.get('has_yellow_tape', False):
                    label += ' [Tape]'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Draw center point
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # Draw 3D coordinates if available
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 绘制蓝色盒子（显示所有检测到的面）
            blue_box_color = (255, 0, 0)  # 蓝色用于蓝色盒子 (BGR格式)
            face_color = (200, 100, 100)  # 浅蓝色用于单个面
            for blue_box in blue_box_coordinates:
                obj_id = blue_box['id']
                bbox = blue_box['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # 绘制合并后的完整边界框（粗线）
                cv2.rectangle(annotated, (x, y), (x_max, y_max), blue_box_color, 3)
                
                # 绘制每个检测到的面（细线）
                num_faces = blue_box.get('num_faces', 1)
                faces = blue_box.get('faces', [])
                if len(faces) > 0:
                    # 绘制每个面的轮廓
                    for face_idx, face in enumerate(faces):
                        contour = face.get('contour')
                        if contour is not None:
                            # 调整轮廓坐标（加上裁剪偏移）
                            adjusted_contour = contour + np.array([20, 20])
                            cv2.drawContours(annotated, [adjusted_contour], -1, face_color, 1)
                
                # 绘制标签（显示面数）
                label = f'Blue Box {obj_id}'
                if num_faces > 1:
                    label += f' ({num_faces} faces)'
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), blue_box_color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 绘制中心点
                cx = blue_box['center']['x']
                cy = blue_box['center']['y']
                cv2.circle(annotated, (cx, cy), 5, blue_box_color, -1)
                cv2.circle(annotated, (cx, cy), 15, blue_box_color, 2)
                
                # 如果可用，绘制3D坐标和尺寸
                point_3d = blue_box.get('point_3d')
                size_3d = blue_box.get('size_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    if size_3d:
                        coord_text += f" ({size_3d['width']:.2f}x{size_3d['height']:.2f}x{size_3d['depth']:.2f}m)"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, blue_box_color, 1)
            
            # Display total count
            total_text = f"Leaves: {len(leaf_coordinates)}"
            if len(blue_box_coordinates) > 0:
                total_text += f" | Blue Boxes: {len(blue_box_coordinates)}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return annotated
            
        except Exception as e:
            self.get_logger().error(f'✗ Annotation drawing error: {str(e)}')
            return display_frame.copy()

