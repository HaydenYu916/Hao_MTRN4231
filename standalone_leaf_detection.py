#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立叶子检测和显示脚本
不依赖ROS2，直接从RealSense相机读取图像并检测叶子
"""

import cv2
import numpy as np
from plantcv import plantcv as pcv
import pyrealsense2 as rs
from datetime import datetime
import argparse
import sys


class StandaloneLeafDetector:
    """独立的叶子检测器，不依赖ROS2"""
    
    def __init__(self, min_area=2000, detect_yellow=True):
        """
        初始化检测器
        
        Args:
            min_area: 最小叶子面积阈值
            detect_yellow: 是否检测黄色区域（用于检测贴有黄色胶布的叶子）
        """
        self.min_area = min_area
        self.detect_yellow = detect_yellow
        
        # PlantCV设置
        pcv.params.debug = None  # 禁用调试输出
        
        # 相机内参（将从RealSense获取）
        self.intrinsics = None
        
        print('✓ 独立叶子检测器初始化完成')
        if self.detect_yellow:
            print('  ✓ 黄色胶布检测已启用')
    
    def setup_camera(self):
        """设置RealSense相机"""
        # 配置深度和颜色流
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # 启用颜色和深度流
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 创建对齐器（将深度对齐到颜色）
        align_to = rs.align(rs.stream.color)
        self.align = align_to
        
        # 启动流
        profile = self.pipeline.start(config)
        
        # 获取颜色流的内参
        color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
        self.intrinsics = color_profile.get_intrinsics()
        
        print(f'✓ RealSense相机初始化完成')
        print(f'  分辨率: {self.intrinsics.width}x{self.intrinsics.height}')
        print(f'  内参: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}')
        
        return True
    
    def get_frames(self):
        """获取同步的颜色和深度帧"""
        try:
            frames = self.pipeline.wait_for_frames()
            
            # 对齐深度帧到颜色帧
            aligned_frames = self.align.process(frames)
            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not aligned_depth_frame or not color_frame:
                return None, None
            
            # 转换为numpy数组
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return color_image, depth_image
        except Exception as e:
            print(f'✗ 获取帧失败: {e}')
            return None, None
    
    def pixel_to_3d(self, pixel_u, pixel_v, depth_value_mm):
        """
        将像素坐标和深度值转换为3D坐标（相机坐标系）
        
        Args:
            pixel_u: 像素u坐标
            pixel_v: 像素v坐标
            depth_value_mm: 深度值（毫米）
        
        Returns:
            3D坐标元组 (x, y, z) 单位：米，如果失败返回None
        """
        if self.intrinsics is None or depth_value_mm == 0:
            return None
        
        try:
            depth_m = depth_value_mm * 0.001  # 转换为米
            point_3d = rs.rs2_deproject_pixel_to_point(
                self.intrinsics,
                [pixel_u, pixel_v],
                depth_m
            )
            return tuple(point_3d)
        except Exception as e:
            print(f'✗ 3D反投影错误: {e}')
            return None
    
    def detect_leaves(self, cv_image, depth_image, frame_count=0):
        """
        使用PlantCV检测叶子
        
        Args:
            cv_image: BGR图像
            depth_image: 深度图像（16位，单位：毫米）
            frame_count: 帧计数（用于调试）
        
        Returns:
            (detection_result_string, leaf_data_dict, bounding_boxes_list, debug_images_dict)
        """
        debug_images = {}  # 存储中间处理步骤的图像
        
        try:
            h, w = cv_image.shape[:2]
            
            # 保存原始图像
            debug_images['original'] = cv_image.copy()
            
            # 轻微裁剪
            crop_img = pcv.crop(img=cv_image, x=20, y=20, h=h-40, w=w-40)
            debug_images['cropped'] = crop_img.copy()
            
            # HSV颜色空间 - 先检测绿色叶子
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            debug_images['hsv'] = hsv.copy()
            
            # 步骤1: 检测绿色叶子
            lower_green = np.array([40, 60, 40])
            upper_green = np.array([80, 255, 255])
            thresh_green = cv2.inRange(hsv, lower_green, upper_green)
            
            # 步骤2: 检测黄色胶布区域（全图检测，用于后续在叶子区域内分析）
            thresh_yellow_full = np.zeros_like(thresh_green)
            if self.detect_yellow:
                lower_yellow = np.array([20, 80, 80])
                upper_yellow = np.array([40, 255, 255])
                thresh_yellow_full = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # 形态学处理：去除小的噪声点
                kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
                thresh_yellow_full = cv2.morphologyEx(thresh_yellow_full, cv2.MORPH_OPEN, kernel_small, iterations=1)
            
            # 先使用绿色检测进行初步处理
            thresh = thresh_green.copy()
            thresh_binary = (thresh / 255).astype(np.uint8)
            
            # 创建调试图像：显示绿色和黄色检测结果
            debug_thresh = np.zeros((crop_img.shape[0], crop_img.shape[1], 3), dtype=np.uint8)
            debug_thresh[:, :, 1] = thresh_green  # 绿色通道显示绿色检测
            debug_thresh[:, :, 0] = thresh_yellow_full  # 蓝色通道显示黄色检测
            debug_images['threshold'] = debug_thresh
            
            # 形态学处理
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            thresh_morph = cv2.morphologyEx(thresh_binary, cv2.MORPH_CLOSE, kernel, iterations=2)
            thresh_morph = cv2.morphologyEx(thresh_morph, cv2.MORPH_OPEN, kernel, iterations=1)
            debug_images['morphology'] = cv2.cvtColor(thresh_morph * 255, cv2.COLOR_GRAY2BGR)
            
            # 轮廓检测
            contours, _ = cv2.findContours(thresh_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 绘制所有轮廓（用于调试）
            contour_img = crop_img.copy()
            cv2.drawContours(contour_img, contours, -1, (0, 255, 255), 2)
            debug_images['contours_all'] = contour_img.copy()
            
            if len(contours) == 0:
                # 即使没有轮廓，也保存空的valid contours图像
                debug_images['contours_valid'] = crop_img.copy()
                return "未检测到叶子", None, None, debug_images
            
            # 过滤参数
            min_area_threshold = self.min_area if self.min_area > 0 else 2000
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
            
            # 绘制有效轮廓（在轮廓过滤后）
            valid_contour_img = crop_img.copy()
            cv2.drawContours(valid_contour_img, valid_contours, -1, (0, 255, 0), 2)
            
            # 添加文本显示轮廓数量（放在右下角，不遮挡）
            h_img, w_img = valid_contour_img.shape[:2]
            text = f'Valid: {len(valid_contours)}'
            text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            text_x = w_img - text_size[0] - 10
            text_y = h_img - 10
            cv2.rectangle(valid_contour_img, (text_x - 5, text_y - text_size[1] - 5),
                         (text_x + text_size[0] + 5, text_y + 5), (0, 0, 0), -1)
            cv2.putText(valid_contour_img, text, (text_x, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            debug_images['contours_valid'] = valid_contour_img.copy()
            
            if len(valid_contours) == 0:
                return "未检测到有效叶子", None, None, debug_images
            
            # 步骤3: 对于每个检测到的绿色叶子，在其区域内检测黄色胶布
            # 同时检测独立的黄色区域（如果形状符合叶子特征）
            leaf_coordinates = []
            bounding_boxes = []
            leaf_idx = 1
            
            # 调试信息
            if frame_count <= 5:
                print(f'  开始处理: {len(valid_contours)} 个绿色轮廓')
            
            # 处理绿色叶子轮廓
            for cnt in valid_contours:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # 计算中心点
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00']) + 20
                    cy = int(M['m01'] / M['m00']) + 20
                else:
                    cx = x + w_rect // 2 + 20
                    cy = y + h_rect // 2 + 20
                
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # 获取深度值和3D坐标（使用更大的区域以提高稳定性）
                depth_value_mm = 0
                point_3d = None
                has_valid_depth = False
                
                if depth_image is not None:
                    try:
                        if cx < depth_image.shape[1] and cy < depth_image.shape[0]:
                            # 使用5x5区域的平均值，提高稳定性
                            y_start = max(0, cy - 2)
                            y_end = min(depth_image.shape[0], cy + 3)
                            x_start = max(0, cx - 2)
                            x_end = min(depth_image.shape[1], cx + 3)
                            
                            depth_region = depth_image[y_start:y_end, x_start:x_end]
                            # 过滤掉无效深度值（0值）
                            valid_depths = depth_region[depth_region > 0]
                            
                            if len(valid_depths) > 0:
                                depth_value_mm = int(np.mean(valid_depths))
                                
                                # 进一步放宽深度范围检查
                                if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                    has_valid_depth = True
                                    point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                            else:
                                # 如果没有有效深度，尝试使用轮廓区域内的深度
                                # 在轮廓边界框内采样多个点
                                bbox_x_start = max(0, x + 20 - 10)
                                bbox_x_end = min(depth_image.shape[1], x + 20 + w_rect + 10)
                                bbox_y_start = max(0, y + 20 - 10)
                                bbox_y_end = min(depth_image.shape[0], y + 20 + h_rect + 10)
                                
                                bbox_depth_region = depth_image[bbox_y_start:bbox_y_end, bbox_x_start:bbox_x_end]
                                bbox_valid_depths = bbox_depth_region[bbox_depth_region > 0]
                                
                                if len(bbox_valid_depths) > 0:
                                    depth_value_mm = int(np.mean(bbox_valid_depths))
                                    if depth_value_mm > 0 and 30 <= depth_value_mm <= 5000:
                                        has_valid_depth = True
                                        point_3d = self.pixel_to_3d(cx, cy, depth_value_mm)
                    except Exception as e:
                        # 调试信息：只在第一帧打印
                        if frame_count <= 3 and leaf_idx == 1:
                            print(f'  深度获取错误: {e}')
                        pass
                
                # 如果深度过滤失败，但仍然保留轮廓（用于调试，至少能看到检测框）
                # 但标记为无深度信息
                if not has_valid_depth:
                    # 调试信息：显示为什么被过滤
                    if frame_count <= 5:  # 前5帧打印调试信息
                        print(f'  轮廓 {leaf_idx} 深度无效: cx={cx}, cy={cy}, depth={depth_value_mm}mm, 尝试使用轮廓区域')
                    
                    # 即使深度无效，也保留轮廓（但标记深度为0）
                    # 这样至少能看到检测框
                    depth_value_mm = 0
                    point_3d = None
                    # 不continue，继续处理这个轮廓
                
                # 检查这个叶子区域内是否包含黄色胶布
                has_yellow_tape = False
                yellow_ratio = 0.0
                
                if self.detect_yellow and thresh_yellow_full is not None:
                    # 使用叶子轮廓创建精确掩码，避免将背景纳入统计
                    leaf_mask = np.zeros_like(thresh_green)
                    cv2.drawContours(leaf_mask, [cnt], -1, 255, -1)
                    
                    # 获取叶子边界框区域（扩大一点以包含边缘）
                    bbox_x_start = max(0, x)
                    bbox_x_end = min(crop_img.shape[1], x + w_rect)
                    bbox_y_start = max(0, y)
                    bbox_y_end = min(crop_img.shape[0], y + h_rect)
                    
                    # 提取该区域的绿色、黄色及叶子掩码
                    leaf_region_mask = leaf_mask[bbox_y_start:bbox_y_end, bbox_x_start:bbox_x_end]
                    leaf_region_yellow = thresh_yellow_full[bbox_y_start:bbox_y_end, bbox_x_start:bbox_x_end]
                    
                    # 计算黄色区域占叶子区域的比例
                    leaf_pixels = np.sum(leaf_region_mask > 0)
                    yellow_pixels = np.sum((leaf_region_yellow > 0) & (leaf_region_mask > 0))
                    
                    if leaf_pixels > 0:
                        yellow_ratio = yellow_pixels / leaf_pixels
                        # 如果黄色区域占比超过5%，认为贴有黄色胶布
                        if yellow_ratio > 0.05:
                            has_yellow_tape = True
                
                # 保存叶子信息
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
            
            # 调试信息
            if frame_count <= 5:
                print(f'  深度过滤后: {num_detected_leaves} 片叶子（原始: {len(valid_contours)} 个轮廓）')
            
            if num_detected_leaves == 0:
                # 即使深度过滤失败，也显示轮廓（用于调试）
                # 创建一个显示所有有效轮廓的结果图像
                result_img = crop_img.copy()
                cv2.drawContours(result_img, valid_contours, -1, (0, 255, 255), 2)  # 黄色轮廓
                
                # 添加调试信息
                h_img, w_img = result_img.shape[:2]
                cv2.putText(result_img, f'No depth info, showing contours', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.putText(result_img, f'Valid contours: {len(valid_contours)}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 为每个轮廓添加编号
                for idx, cnt in enumerate(valid_contours):
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M['m10'] / M['m00']) + 20
                        cy = int(M['m01'] / M['m00']) + 20
                        cv2.putText(result_img, f'{idx+1}', (cx-10, cy),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                
                debug_images['result'] = result_img
                return "未检测到有效叶子（深度过滤后）", None, None, debug_images
            
            result = f"检测到 {num_detected_leaves} 片叶子"
            leaf_data = {
                'num_leaves': num_detected_leaves,
                'timestamp': datetime.now().isoformat(),
                'coordinates': leaf_coordinates
            }
            
            return result, leaf_data, bounding_boxes, debug_images
            
        except Exception as e:
            print(f'✗ PlantCV检测错误: {str(e)}')
            import traceback
            traceback.print_exc()
            return f"检测错误: {str(e)}", None, None, debug_images
    
    def draw_annotations(self, display_frame, leaf_coordinates):
        """
        在图像上绘制标注
        
        Args:
            display_frame: 要绘制的图像
            leaf_coordinates: 叶子坐标列表
        
        Returns:
            标注后的图像
        """
        if display_frame is None:
            return None
        
        # 始终返回帧的副本（带或不带标注）
        annotated = display_frame.copy()
        
        if not leaf_coordinates:
            # 未检测到叶子，只返回原始帧
            return annotated
        
        # 为检测到的叶子绘制标注
        try:
            colors = [
                (255, 0, 0),      # 蓝色
                (0, 255, 0),       # 绿色
                (0, 0, 255),       # 红色
                (255, 255, 0),     # 青色
                (255, 0, 255),     # 洋红色
                (0, 255, 255),     # 黄色
            ]
            
            for leaf in leaf_coordinates:
                obj_id = leaf['id']
                color = colors[(obj_id - 1) % len(colors)]
                
                bbox = leaf['bounding_box']
                x = bbox['x_min']
                y = bbox['y_min']
                x_max = bbox['x_max']
                y_max = bbox['y_max']
                
                # 绘制边界框
                cv2.rectangle(annotated, (x, y), (x_max, y_max), color, 2)
                
                # 绘制标签（如果有胶布，添加标记）
                label = f'Leaf {obj_id}'
                has_tape = leaf.get('has_yellow_tape', False)
                if has_tape:
                    label += ' [Tape]'
                
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                label_y = max(y - 5, label_size[1] + 5)
                
                cv2.rectangle(annotated, (x, label_y - label_size[1] - 5),
                            (x + label_size[0] + 5, label_y + 5), color, -1)
                cv2.putText(annotated, label, (x + 2, label_y - 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # 绘制中心点
                cx = leaf['center']['x']
                cy = leaf['center']['y']
                cv2.circle(annotated, (cx, cy), 5, color, -1)
                cv2.circle(annotated, (cx, cy), 15, color, 2)
                
                # 如果可用，绘制3D坐标
                point_3d = leaf.get('point_3d')
                if point_3d:
                    coord_text = f"Z:{point_3d[2]:.2f}m"
                    cv2.putText(annotated, coord_text,
                               (x + 5, y_max - 5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            
            # 显示总数
            total_text = f"Leaves: {len(leaf_coordinates)}"
            cv2.putText(annotated, total_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return annotated
            
        except Exception as e:
            print(f'✗ 标注绘制错误: {e}')
            import traceback
            traceback.print_exc()
            return display_frame.copy()
    
    def create_debug_mosaic(self, debug_images):
        """
        创建包含所有处理步骤的拼接图像
        
        Args:
            debug_images: 包含各个处理步骤图像的字典
        
        Returns:
            拼接后的大图像
        """
        # 定义图像顺序和标签
        image_order = [
            ('original', '1. Original'),
            ('cropped', '2. Cropped'),
            ('hsv', '3. HSV'),
            ('threshold', '4. Threshold'),
            ('morphology', '5. Morphology'),
            ('contours_all', '6. All Contours'),
            ('contours_valid', '7. Valid Contours'),
            ('result', '8. Final Result')
        ]
        
        # 每个小图像的大小
        tile_width = 320
        tile_height = 240
        
        # 创建3x3网格（最后一行可能只有2个）
        rows = 3
        cols = 3
        
        # 创建大画布
        mosaic_height = rows * tile_height
        mosaic_width = cols * tile_width
        mosaic = np.zeros((mosaic_height, mosaic_width, 3), dtype=np.uint8)
        
        # 填充每个位置
        for idx, (key, label) in enumerate(image_order):
            row = idx // cols
            col = idx % cols
            
            if key in debug_images and debug_images[key] is not None:
                img = debug_images[key]
                if img.size > 0:
                    # 调整图像大小
                    resized = cv2.resize(img, (tile_width, tile_height))
                    
                    # 计算位置
                    y_start = row * tile_height
                    y_end = y_start + tile_height
                    x_start = col * tile_width
                    x_end = x_start + tile_width
                    
                    # 放置图像
                    mosaic[y_start:y_end, x_start:x_end] = resized
                    
                    # 添加标签
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(mosaic, label, (x_start + 5, y_start + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
        return mosaic
    
    def run(self):
        """运行检测循环"""
        print('\n开始检测循环...')
        print('按 \'q\' 键退出\n')
        
        frame_count = 0
        window_name = 'Leaf Detection - All Processing Steps'
        
        # 创建单个大窗口
        try:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 960, 720)  # 3x3网格，每个320x240
            print('✓ 可视化窗口已创建')
            print('   所有处理步骤将显示在一个窗口中')
        except Exception as e:
            print(f'⚠ 窗口创建警告: {e}')
            print('  继续运行，但窗口可能无法显示')
        
        try:
            while True:
                # 获取帧
                color_image, depth_image = self.get_frames()
                
                if color_image is None or depth_image is None:
                    if frame_count == 0:
                        print('⚠ 等待相机数据...')
                    continue
                
                frame_count += 1
                
                # 检测叶子（传递frame_count用于调试）
                detection_result, leaf_data, bounding_boxes, debug_images = self.detect_leaves(
                    color_image, depth_image, frame_count
                )
                
                # 绘制标注
                if leaf_data and leaf_data.get('num_leaves', 0) > 0:
                    annotated_image = self.draw_annotations(
                        color_image,
                        leaf_data.get('coordinates', [])
                    )
                    
                    # 打印检测信息
                    if frame_count % 30 == 0:  # 每30帧打印一次
                        print(f'帧 {frame_count}: {detection_result}')
                        for i, leaf in enumerate(leaf_data.get('coordinates', [])):
                            point_3d = leaf.get('point_3d')
                            if point_3d:
                                print(f'  Leaf {i+1}: X={point_3d[0]:.3f}m, '
                                      f'Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m')
                else:
                    annotated_image = color_image.copy()
                
                # 保存最终结果到调试图像
                debug_images['result'] = annotated_image.copy()
                
                # 检查图像是否有效
                if annotated_image is None or annotated_image.size == 0:
                    print('⚠ 图像无效，跳过显示')
                    continue
                
                # 创建拼接图像
                try:
                    mosaic = self.create_debug_mosaic(debug_images)
                    
                    # 显示拼接后的图像
                    cv2.imshow(window_name, mosaic)
                    
                    # 首次显示时打印提示
                    if frame_count == 1:
                        print('✓ 图像显示已启动')
                        print('   所有处理步骤显示在一个窗口中')
                        print('   如果看不到窗口，请检查：')
                        print('   1. 窗口是否被其他窗口遮挡')
                        print('   2. 是否在远程SSH连接（需要X11转发）')
                        print('   3. DISPLAY环境变量是否正确设置')
                except Exception as e:
                    print(f'✗ 显示图像失败: {e}')
                    import traceback
                    traceback.print_exc()
                    print('   提示: 如果使用SSH，请使用 X11 转发: ssh -X user@host')
                    break
                
                # 按'q'退出
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print('\n用户按下 \'q\' 键，退出程序')
                    break
                elif key == 27:  # ESC键
                    print('\n用户按下 ESC 键，退出程序')
                    break
                
        except KeyboardInterrupt:
            print('\n用户中断 (Ctrl+C)')
        except Exception as e:
            print(f'\n✗ 运行错误: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # 清理
            try:
                cv2.destroyAllWindows()
            except:
                pass
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
            print('\n✓ 程序退出')


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='独立叶子检测和显示脚本（不依赖ROS2）'
    )
    parser.add_argument(
        '--min-area',
        type=float,
        default=2000,
        help='最小叶子面积阈值（默认: 2000）'
    )
    parser.add_argument(
        '--no-yellow',
        action='store_true',
        help='禁用黄色胶布检测（只检测绿色叶子）'
    )
    
    args = parser.parse_args()
    
    # 创建检测器
    detector = StandaloneLeafDetector(
        min_area=args.min_area,
        detect_yellow=not args.no_yellow
    )
    
    # 设置相机
    try:
        detector.setup_camera()
    except Exception as e:
        print(f'✗ 相机初始化失败: {e}')
        print('请确保RealSense相机已连接并正确配置')
        sys.exit(1)
    
    # 运行检测循环
    detector.run()


if __name__ == '__main__':
    main()

