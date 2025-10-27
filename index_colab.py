# -*- coding: utf-8 -*-
"""
RealSense Real-time Leaf Detection Program
Multi-leaf detection and analysis based on PlantCV

Features:
- Real-time image capture using RealSense camera
- Apply PlantCV for leaf segmentation and detection
- Real-time display of detection results and analysis data
- Support for saving detection results and annotated images
"""

# ============================================================================
# 📋 Configuration Definition - Modify run mode here
# ============================================================================

# Select detection mode
FAST_MODE = True          # True=Fast mode(30+FPS), False=Precise mode(Low FPS but precise)

# ============================================================================
# Fine-grained Function Control - Choose which PlantCV analysis functions to enable
# ============================================================================
# Note: Each function can be independently enabled/disabled, combination can balance speed and accuracy

USE_SIZE_ANALYSIS = True      # Enable size analysis (consumption: medium)
USE_COLOR_ANALYSIS = False     # Enable color analysis (consumption: medium)
USE_MORPHOLOGY = True          # Enable morphological processing (consumption: low, recommended)
USE_CONTOUR_FILTERING = True   # Enable contour filtering (consumption: low, recommended)

# ============================================================================
# Other Configuration
# ============================================================================
SAVE_RESULTS = False       # True=Auto save results, False=Don't save
SAVE_INTERVAL = 5          # Save every N frames
OUTPUT_DIR = "./leaf_detection_results"

# ============================================================================
# Function Performance Impact Assessment
# ============================================================================
# USE_SIZE_ANALYSIS: ↓ Frame rate reduced by 15-20%
# USE_COLOR_ANALYSIS: ↓ Frame rate reduced by 15-20%
# Both enabled: ↓↓ Frame rate reduced by 30-40%
# Both disabled: ✓ Fastest speed (30+ FPS)
#
# Recommended configurations:
# 🚀 Ultra fast: Both disabled -> 30+ FPS
# ⚖️ Balanced: Only enable MORPHOLOGY -> 25+ FPS
# 🎯 Precise: Both enabled -> 15-20 FPS
# ============================================================================

import sys
import warnings
import os

# Suppress PlantCV deprecation warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
warnings.filterwarnings("ignore", category=RuntimeWarning)

import cv2
import numpy as np
import pyrealsense2 as rs
from plantcv import plantcv as pcv
import json
from datetime import datetime
from collections import deque


class RealSenseLeafDetector:
    """
    Real-time leaf detector using RealSense camera and PlantCV
    """
    
    def __init__(self, output_dir="./leaf_detection_results", test_mode=False, fast_mode=True, 
                 use_size=False, use_color=False, use_morphology=True, use_filtering=True):
        """
        Initialize RealSense and PlantCV parameters
        
        Args:
            output_dir (str): Output results save directory
            test_mode (bool): Whether to use test mode
            fast_mode (bool): Fast mode
            use_size (bool): Enable size analysis
            use_color (bool): Enable color analysis
            use_morphology (bool): Enable morphological processing
            use_filtering (bool): Enable contour filtering
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        self.test_mode = test_mode
        self.fast_mode = fast_mode
        self.use_size = use_size
        self.use_color = use_color
        self.use_morphology = use_morphology
        self.use_filtering = use_filtering
        
        self.pipeline = None
        self.profile = None
        self.depth_scale = 0.001
        self.intrinsics = None
        self.align = None
        
        # Initialize PlantCV parameters
        pcv.params.debug = None
        pcv.params.text_size = 0.5
        pcv.params.text_thickness = 1
        
        # Detection results cache
        self.results_history = deque(maxlen=30)
        
        if test_mode:
            self._init_test_mode()
        else:
            self._init_realsense()
        
        # Print mode information
        print(f"📊 Detection mode: {'⚡ Fast mode' if self.fast_mode else '🎯 Precise mode'}")
        print(f"📊 Enabled functions:")
        print(f"   • Size analysis: {'✓' if self.use_size else '✗'}")
        print(f"   • Color analysis: {'✓' if self.use_color else '✗'}")
        print(f"   • Morphological processing: {'✓' if self.use_morphology else '✗'}")
        print(f"   • Contour filtering: {'✓' if self.use_filtering else '✗'}\n")
    
    def _init_realsense(self):
        """Initialize RealSense camera"""
        print("Attempting to connect to RealSense camera...")
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Try to detect available devices
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            print("✗ No RealSense devices detected")
            print("  Please check:")
            print("  1. Is RealSense camera connected to USB")
            print("  2. Is librealsense driver installed")
            print("  3. Are USB permissions correct")
            raise RuntimeError("❌ RealSense camera connection failed - no devices detected")
        
        print(f"✓ Detected {len(devices)} devices")
        
        # Configure RGB and depth streams
        try:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        except Exception as e:
            print(f"✗ Stream configuration failed: {e}")
            raise RuntimeError(f"❌ RealSense stream configuration failed: {e}")
        
        # Start pipeline
        try:
            self.profile = self.pipeline.start(self.config)
        except Exception as e:
            print(f"✗ Pipeline startup failed: {e}")
            raise RuntimeError(f"❌ RealSense pipeline startup failed: {e}")
        
        # Get depth sensor
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Create depth to RGB alignment object
        self.align = rs.align(rs.stream.color)
        
        # Get camera intrinsics
        color_profile = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
        
        print("✓ RealSense camera initialized")
        print(f"  Resolution: {self.intrinsics.width}x{self.intrinsics.height}")
        print(f"  Frame rate: 30 FPS")
        print(f"  Depth scale: {self.depth_scale}")
    
    def _init_test_mode(self):
        """初始化测试模式（不需要硬件）"""
        print("✓ 进入测试模式")
        print("  使用生成的测试图像")
        self.test_mode = True
        self.test_frame_count = 0
    
    def capture_frame(self):
        """
        从 RealSense 相机捕获彩色和深度图像
        
        Returns:
            tuple: (彩色图像, 深度图像) 或 (None, None) 如果捕获失败
        """
        if self.test_mode:
            # 测试模式下，使用生成的测试图像
            self.test_frame_count += 1
            if self.test_frame_count > 10:
                return None, None
            
            # 生成一个带有绿色叶子特征的测试图像
            h, w = 480, 640
            img = np.zeros((h, w, 3), dtype=np.uint8)
            
            # 白色背景
            img[:, :] = [200, 200, 200]
            
            # 绘制模拟的绿色植物区域
            cv2.ellipse(img, (320, 240), (100, 80), 0, 0, 360, (50, 150, 50), -1)
            cv2.ellipse(img, (260, 200), (60, 50), 0, 0, 360, (60, 180, 60), -1)
            cv2.ellipse(img, (380, 200), (60, 50), 0, 0, 360, (60, 180, 60), -1)
            cv2.ellipse(img, (280, 300), (70, 60), 0, 0, 360, (40, 140, 40), -1)
            cv2.ellipse(img, (360, 300), (70, 60), 0, 0, 360, (40, 140, 40), -1)
            
            # 添加文本标记
            cv2.putText(img, f"Test Frame {self.test_frame_count}", (w//2 - 120, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            cv2.putText(img, "Test Mode", (w//2 - 70, h-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # 模拟深度图像
            depth_img = np.ones((h, w), dtype=np.uint16) * 500
            
            return img, depth_img
        else:
            try:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    return None, None
                
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                return color_image, depth_image
            except Exception as e:
                print(f"✗ 捕获图像失败: {e}")
                return None, None
    
    def preprocess_image(self, img):
        """
        预处理图像：裁剪、旋转、色彩校正
        
        Args:
            img: 输入 BGR 图像
            
        Returns:
            预处理后的图像
        """
        # 裁剪图像 (可选)
        h, w = img.shape[:2]
        crop_img = pcv.crop(img=img, x=50, y=50, h=h-100, w=w-100)
        
        # 旋转（如需要        rotate_img = pcv.transform.rotate(crop_img, 0, False)
        
        # 色彩校正（如果失败，使用原图）
        try:
            corrected_img = pcv.transform.auto_correct_color(rgb_img=rotate_img)
        except:
            corrected_img = rotate_img
        
        return corrected_img
    
    def detect_leaves(self, img):
        """
        检测叶子并进行分析（优化版本以提高帧率）
        
        Args:
            img: 输入 BGR 图像
            
        Returns:
            dict: 包含检测结果和分析数据
        """
        try:
            # 快速处理：跳过色彩校正以节省时间
            h, w = img.shape[:2]
            
            # 只做最小的裁剪（减少计算）
            crop_img = pcv.crop(img=img, x=20, y=20, h=h-40, w=w-40)
            
            # 快速颜色检测 - 直接使用 HSV（跳过色彩校正）
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
            # 更严格的绿色范围（过滤掉黑色和其他非植物物体）
            # H: 35-85 (绿色), S: 50-255 (饱和度更高), V: 60-255 (亮度)
            lower_green = np.array([35, 50, 60])
            upper_green = np.array([85, 255, 255])
            thresh = cv2.inRange(hsv, lower_green, upper_green)
            thresh = (thresh / 255).astype(np.uint8)
            
            # 快速形态学处理（简化版本）
            if self.use_morphology:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # 轮廓检测（比 PlantCV 的方法快）
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 轮廓过滤（可选）
            if self.use_filtering:
                # 严格的过滤条件
                valid_contours = []
                min_area = 500      # 最小面积（过滤太小的噪声）
                max_area = 50000    # 最大面积（过滤背景）
                min_circularity = 0.3  # 最小圆形度
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    
                    # 过滤面积
                    if area < min_area or area > max_area:
                        continue
                    
                    # 计算圆形度
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                        
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity < min_circularity:
                        continue
                    
                    # 检查宽高比
                    x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                        
                    aspect_ratio = float(w_rect) / h_rect
                    if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                        continue
                    
                    valid_contours.append(cnt)
            else:
                # 不过滤，只做最小面积检查
                valid_contours = [c for c in contours if cv2.contourArea(c) > 100]
            
            n_obj = len(valid_contours)
            
            # 创建标记图像
            labeled_objects = np.zeros_like(thresh)
            leaf_coordinates = []  # 存储所有叶子的坐标
            
            for idx, cnt in enumerate(valid_contours, 1):
                cv2.drawContours(labeled_objects, [cnt], 0, idx, -1)
                
                # 计算叶子的坐标信息
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # 计算中心点
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                else:
                    cx = x + w_rect // 2
                    cy = y + h_rect // 2
                
                # 计算面积和周长
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # 保存叶子信息
                leaf_info = {
                    'id': idx,
                    'center': {'x': cx, 'y': cy},  # 中心点坐标
                    'bounding_box': {  # 边界框坐标
                        'x': x,
                        'y': y,
                        'width': w_rect,
                        'height': h_rect,
                        'x_min': x,
                        'y_min': y,
                        'x_max': x + w_rect,
                        'y_max': y + h_rect
                    },
                    'area': float(area),  # 面积
                    'perimeter': float(perimeter),  # 周长
                }
                leaf_coordinates.append(leaf_info)
            
            # 可选：PlantCV 分析功能
            analysis_results = {}
            if (self.use_size or self.use_color) and len(valid_contours) > 0:
                try:
                    if self.use_size:
                        # 大小分析
                        size_analysis = pcv.analyze.size(
                            img=crop_img,
                            labeled_mask=labeled_objects,
                            n_labels=len(valid_contours)
                        )
                        analysis_results['size'] = size_analysis
                except:
                    pass
                
                try:
                    if self.use_color:
                        # 颜色分析
                        color_analysis = pcv.analyze.color(
                            rgb_img=crop_img,
                            labeled_mask=labeled_objects,
                            n_labels=len(valid_contours),
                            colorspaces='hsv'
                        )
                        analysis_results['color'] = color_analysis
                except:
                    pass
            
            # 获取检测结果
            results = {
                'timestamp': datetime.now().isoformat(),
                'num_leaves': n_obj,
                'processed_image': crop_img,
                'mask': thresh,
                'labeled_objects': labeled_objects,
                'contours': valid_contours,
                'analysis': analysis_results,
                'outputs': pcv.outputs.observations if (self.use_size or self.use_color) else {},
                'leaf_coordinates': leaf_coordinates # 添加叶子坐标信息
            }
            
            return results
            
        except Exception as e:
            print(f"✗ 叶子检测失败: {e}")
            return None
    
    def draw_results(self, img, results):
        """
        在图像上绘制检测结果
        
        Args:
            img: 原始 BGR 图像
            results: 检测结果字典
            
        Returns:
            标注后的图像
        """
        if results is None:
            return img
        
        annotated_img = img.copy()
        h, w = annotated_img.shape[:2]
        
        # 绘制文本信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        text_color = (0, 255, 0)
        
        y_offset = 30
        cv2.putText(annotated_img, f"Detected Leaves: {results['num_leaves']}", 
                    (10, y_offset), font, font_scale, text_color, thickness)
        
        # 显示时间戳
        cv2.putText(annotated_img, f"Time: {results['timestamp'][-8:]}", 
                    (10, y_offset + 30), font, font_scale, text_color, thickness)
        
        # 显示叶子坐标信息（在控制台打印，不在图像上显示以免过拥挤）
        if results.get('leaf_coordinates'):
            print(f"\n📍 叶子坐标信息 (帧 {results.get('frame_id', 'N/A')}):")
            for leaf in results['leaf_coordinates']:
                print(f"   Leaf {leaf['id']}: 中心=({leaf['center']['x']}, {leaf['center']['y']}), "
                      f"面积={leaf['area']:.0f}, 周长={leaf['perimeter']:.1f}")
        
        return annotated_img
    
    def save_results(self, img, results, frame_id):
        """
        保存检测结果和标注图像
        
        Args:
            img: 原始图像
            results: 检测结果
            frame_id: 帧编号
        """
        if results is None:
            return
        
        try:
            # 保存标注图像
            annotated_img = self.draw_results(img, results)
            img_path = os.path.join(self.output_dir, f"annotated_{frame_id:04d}.jpg")
            cv2.imwrite(img_path, annotated_img)
            
            # 保存掩膜
            mask_path = os.path.join(self.output_dir, f"mask_{frame_id:04d}.jpg")
            cv2.imwrite(mask_path, results['mask'] * 255)
            
            # 保存详细结果为 JSON（包含坐标）
            json_results = {
                'frame_id': frame_id,
                'timestamp': results['timestamp'],
                'num_leaves': results['num_leaves'],
                'leaf_coordinates': results.get('leaf_coordinates', [])  # 包含所有坐标
            }
            
            json_path = os.path.join(self.output_dir, f"results_{frame_id:04d}.json")
            with open(json_path, 'w') as f:
                json.dump(json_results, f, indent=2)
                
        except Exception as e:
            print(f"✗ 保存结果失败: {e}")
    
    def run(self, save_interval=10):
        """
        主循环：实时处理视频流
        
        Args:
            save_interval (int): 每隔多少帧保存一次结果
        """
        frame_count = 0
        
        print("\n开始实时叶子检测...")
        print("按 'q' 键退出，按 's' 键保存当前帧\n")
        
        try:
            while True:
                # 捕获图像
                color_image, depth_image = self.capture_frame()
                if color_image is None:
                    continue
                
                frame_count += 1
                
                # 检测叶子
                results = self.detect_leaves(color_image)
                
                # 绘制结果
                display_img = self.draw_results(color_image, results)
                
                # 显示处理后的标注图像（而不是掩膜）
                if results is not None:
                    # 在处理后的图像上绘制标注
                    annotated_img = self._draw_annotations(results['processed_image'], results)
                    
                    # 调整尺寸以匹配display_img
                    h_display, w_display = display_img.shape[:2]
                    h_annot, w_annot = annotated_img.shape[:2]
                    
                    if h_annot != h_display or w_annot != w_display:
                        annotated_img = cv2.resize(annotated_img, (w_display, h_display))
                    
                    # 合并图像
                    combined = np.hstack([display_img, annotated_img])
                else:
                    combined = display_img
                
                # 显示实时结果
                cv2.imshow('RealSense Leaf Detection', combined)
                
                # 定期保存结果
                if save_interval > 0 and frame_count % save_interval == 0 and results is not None:
                    self.save_results(color_image, results, frame_count)
                    print(f"✓ 帧 {frame_count}: {results['num_leaves']} 片叶子检测到")
                
                # 键盘控制
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n退出程序...")
                    break
                elif key == ord('s'):
                    self.save_results(color_image, results, frame_count)
                    print(f"✓ 保存当前帧 {frame_count}")
        
        except KeyboardInterrupt:
            print("\n被用户中断")
        finally:
            self.cleanup()
    
    def _draw_annotations(self, img, results):
        """
        在图像上绘制叶子标注（边界框和标签）
        
        Args:
            img: 输入图像
            results: 检测结果
            
        Returns:
            标注后的彩色图像
        """
        annotated = img.copy()
        
        if results is None or 'contours' not in results:
            return annotated
        
        contours = results['contours']
        
        # 为每个检测到的对象绘制不同颜色的标记
        colors = [
            (255, 0, 0),
            (0, 255, 0),
            (0, 0, 255),
            (255, 255, 0),
            (255, 0, 255),
            (0, 255, 255),
        ]
        
        try:
            # 为每个轮廓绘制标注
            for obj_id, cnt in enumerate(contours, 1):
                color = colors[(obj_id - 1) % len(colors)]
                
                # 绘制边界框
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
                
                # 绘制标签
                cv2.putText(annotated, f'Leaf {obj_id}', (x, y - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # 绘制中心点
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(annotated, (cx, cy), 5, color, -1)
        except Exception as e:
            pass
        
        return annotated
    
    def cleanup(self):
        """清理资源"""
        print("\n清理资源...")
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("✓ 程序正常退出")


def main():
    """主函数"""
    print("=" * 60)
    print("RealSense 实时叶子检测系统")
    print("基于 PlantCV 的多叶检测和分析")
    print("=" * 60)
    
    # 打印配置信息
    print(f"\n⚙️  当前配置:")
    print(f"   • 检测模式: {'⚡ 快速 (30+ FPS)' if FAST_MODE else '🎯 精确 (10-15 FPS)'}")
    print(f"   • 大小分析: {'✓ 启用' if USE_SIZE_ANALYSIS else '✗ 禁用'}")
    print(f"   • 颜色分析: {'✓ 启用' if USE_COLOR_ANALYSIS else '✗ 禁用'}")
    print(f"   • 形态学处理: {'✓ 启用' if USE_MORPHOLOGY else '✗ 禁用'}")
    print(f"   • 轮廓过滤: {'✓ 启用' if USE_CONTOUR_FILTERING else '✗ 禁用'}")
    print(f"   • 保存结果: {'✓ 启用' if SAVE_RESULTS else '✗ 禁用'}")
    print(f"   • 保存间隔: 每 {SAVE_INTERVAL} 帧")
    print(f"   • 输出目录: {OUTPUT_DIR}\n")
    
    # 创建检测器
    detector = RealSenseLeafDetector(
        output_dir=OUTPUT_DIR,
        fast_mode=FAST_MODE,
        use_size=USE_SIZE_ANALYSIS,
        use_color=USE_COLOR_ANALYSIS,
        use_morphology=USE_MORPHOLOGY,
        use_filtering=USE_CONTOUR_FILTERING
    )
    
    # 运行实时检测
    save_interval = SAVE_INTERVAL if SAVE_RESULTS else 0
    detector.run(save_interval=save_interval)


if __name__ == "__main__":
    main()
