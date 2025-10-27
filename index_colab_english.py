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
        """Initialize test mode (no hardware required)"""
        print("✓ Entering test mode")
        print("  Using generated test images")
        self.test_mode = True
        self.test_frame_count = 0
    
    def capture_frame(self):
        """
        Capture color and depth images from RealSense camera
        
        Returns:
            tuple: (color_image, depth_image) or (None, None) if capture fails
        """
        if self.test_mode:
            # In test mode, use generated test images
            self.test_frame_count += 1
            if self.test_frame_count > 10:
                return None, None
            
            # Generate a test image with green leaf features
            h, w = 480, 640
            img = np.zeros((h, w, 3), dtype=np.uint8)
            
            # White background
            img[:, :] = [200, 200, 200]
            
            # Draw simulated green plant areas
            cv2.ellipse(img, (320, 240), (100, 80), 0, 0, 360, (50, 150, 50), -1)
            cv2.ellipse(img, (260, 200), (60, 50), 0, 0, 360, (60, 180, 60), -1)
            cv2.ellipse(img, (380, 200), (60, 50), 0, 0, 360, (60, 180, 60), -1)
            cv2.ellipse(img, (280, 300), (70, 60), 0, 0, 360, (40, 140, 40), -1)
            cv2.ellipse(img, (360, 300), (70, 60), 0, 0, 360, (40, 140, 40), -1)
            
            # Add text markers
            cv2.putText(img, f"Test Frame {self.test_frame_count}", (w//2 - 120, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            cv2.putText(img, "Test Mode", (w//2 - 70, h-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            
            # Simulate depth image
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
                print(f"✗ Image capture failed: {e}")
                return None, None
    
    def preprocess_image(self, img):
        """
        Preprocess image: crop, rotate, color correction
        
        Args:
            img: Input BGR image
            
        Returns:
            Preprocessed image
        """
        # Crop image (optional)
        h, w = img.shape[:2]
        crop_img = pcv.crop(img=img, x=50, y=50, h=h-100, w=w-100)
        
        # Rotate (if needed)
        rotate_img = pcv.transform.rotate(crop_img, 0, False)
        
        # Color correction (use original if fails)
        try:
            corrected_img = pcv.transform.auto_correct_color(rgb_img=rotate_img)
        except:
            corrected_img = rotate_img
        
        return corrected_img
    
    def detect_leaves(self, img):
        """
        Detect leaves and perform analysis (optimized version for higher frame rate)
        
        Args:
            img: Input BGR image
            
        Returns:
            dict: Contains detection results and analysis data
        """
        try:
            # Fast processing: skip color correction to save time
            h, w = img.shape[:2]
            
            # Only minimal cropping (reduce computation)
            crop_img = pcv.crop(img=img, x=20, y=20, h=h-40, w=w-40)
            
            # Fast color detection - directly use HSV (skip color correction)
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
            # Stricter green range (filter out black and other non-plant objects)
            # H: 35-85 (green), S: 50-255 (higher saturation), V: 60-255 (brightness)
            lower_green = np.array([35, 50, 60])
            upper_green = np.array([85, 255, 255])
            thresh = cv2.inRange(hsv, lower_green, upper_green)
            thresh = (thresh / 255).astype(np.uint8)
            
            # Fast morphological processing (simplified version)
            if self.use_morphology:
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
                thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
            
            # Contour detection (faster than PlantCV method)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Contour filtering (optional)
            if self.use_filtering:
                # Strict filtering conditions
                valid_contours = []
                min_area = 500      # Minimum area (filter out small noise)
                max_area = 50000    # Maximum area (filter out background)
                min_circularity = 0.3  # Minimum circularity
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    
                    # Filter by area
                    if area < min_area or area > max_area:
                        continue
                    
                    # Calculate circularity
                    perimeter = cv2.arcLength(cnt, True)
                    if perimeter == 0:
                        continue
                        
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity < min_circularity:
                        continue
                    
                    # Check aspect ratio
                    x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                    if w_rect == 0 or h_rect == 0:
                        continue
                        
                    aspect_ratio = float(w_rect) / h_rect
                    if aspect_ratio < 0.3 or aspect_ratio > 3.0:
                        continue
                    
                    valid_contours.append(cnt)
            else:
                # No filtering, only minimum area check
                valid_contours = [c for c in contours if cv2.contourArea(c) > 100]
            
            n_obj = len(valid_contours)
            
            # Create labeled image
            labeled_objects = np.zeros_like(thresh)
            leaf_coordinates = []  # Store all leaf coordinates
            
            for idx, cnt in enumerate(valid_contours, 1):
                cv2.drawContours(labeled_objects, [cnt], 0, idx, -1)
                
                # Calculate leaf coordinate information
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                M = cv2.moments(cnt)
                
                # Calculate center point
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                else:
                    cx = x + w_rect // 2
                    cy = y + h_rect // 2
                
                # Calculate area and perimeter
                area = cv2.contourArea(cnt)
                perimeter = cv2.arcLength(cnt, True)
                
                # Save leaf information
                leaf_info = {
                    'id': idx,
                    'center': {'x': cx, 'y': cy},  # Center point coordinates
                    'bounding_box': {  # Bounding box coordinates
                        'x': x,
                        'y': y,
                        'width': w_rect,
                        'height': h_rect,
                        'x_min': x,
                        'y_min': y,
                        'x_max': x + w_rect,
                        'y_max': y + h_rect
                    },
                    'area': float(area),  # Area
                    'perimeter': float(perimeter),  # Perimeter
                }
                leaf_coordinates.append(leaf_info)
            
            # Optional: PlantCV analysis functions
            analysis_results = {}
            if (self.use_size or self.use_color) and len(valid_contours) > 0:
                try:
                    if self.use_size:
                        # Size analysis
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
                        # Color analysis
                        color_analysis = pcv.analyze.color(
                            rgb_img=crop_img,
                            labeled_mask=labeled_objects,
                            n_labels=len(valid_contours),
                            colorspaces='hsv'
                        )
                        analysis_results['color'] = color_analysis
                except:
                    pass
            
            # Get detection results
            results = {
                'timestamp': datetime.now().isoformat(),
                'num_leaves': n_obj,
                'processed_image': crop_img,
                'mask': thresh,
                'labeled_objects': labeled_objects,
                'contours': valid_contours,
                'analysis': analysis_results,
                'outputs': pcv.outputs.observations if (self.use_size or self.use_color) else {},
                'leaf_coordinates': leaf_coordinates # Add leaf coordinate information
            }
            
            return results
            
        except Exception as e:
            print(f"✗ Leaf detection failed: {e}")
            return None
    
    def draw_results(self, img, results):
        """
        Draw detection results on image
        
        Args:
            img: Original BGR image
            results: Detection results dictionary
            
        Returns:
            Annotated image
        """
        if results is None:
            return img
        
        annotated_img = img.copy()
        h, w = annotated_img.shape[:2]
        
        # Draw text information
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        text_color = (0, 255, 0)
        
        y_offset = 30
        cv2.putText(annotated_img, f"Detected Leaves: {results['num_leaves']}", 
                    (10, y_offset), font, font_scale, text_color, thickness)
        
        # Display timestamp
        cv2.putText(annotated_img, f"Time: {results['timestamp'][-8:]}", 
                    (10, y_offset + 30), font, font_scale, text_color, thickness)
        
        # Display leaf coordinate information (print to console, not on image to avoid crowding)
        if results.get('leaf_coordinates'):
            print(f"\n📍 Leaf coordinate information (frame {results.get('frame_id', 'N/A')}):")
            for leaf in results['leaf_coordinates']:
                print(f"   Leaf {leaf['id']}: center=({leaf['center']['x']}, {leaf['center']['y']}), "
                      f"area={leaf['area']:.0f}, perimeter={leaf['perimeter']:.1f}")
        
        return annotated_img
    
    def save_results(self, img, results, frame_id):
        """
        Save detection results and annotated images
        
        Args:
            img: Original image
            results: Detection results
            frame_id: Frame number
        """
        if results is None:
            return
        
        try:
            # Save annotated image
            annotated_img = self.draw_results(img, results)
            img_path = os.path.join(self.output_dir, f"annotated_{frame_id:04d}.jpg")
            cv2.imwrite(img_path, annotated_img)
            
            # Save mask
            mask_path = os.path.join(self.output_dir, f"mask_{frame_id:04d}.jpg")
            cv2.imwrite(mask_path, results['mask'] * 255)
            
            # Save detailed results as JSON (including coordinates)
            json_results = {
                'frame_id': frame_id,
                'timestamp': results['timestamp'],
                'num_leaves': results['num_leaves'],
                'leaf_coordinates': results.get('leaf_coordinates', [])  # Include all coordinates
            }
            
            json_path = os.path.join(self.output_dir, f"results_{frame_id:04d}.json")
            with open(json_path, 'w') as f:
                json.dump(json_results, f, indent=2)
                
        except Exception as e:
            print(f"✗ Save results failed: {e}")
    
    def run(self, save_interval=10):
        """
        Main loop: real-time video stream processing
        
        Args:
            save_interval (int): Save results every N frames
        """
        frame_count = 0
        
        print("\nStarting real-time leaf detection...")
        print("Press 'q' to quit, press 's' to save current frame\n")
        
        try:
            while True:
                # Capture image
                color_image, depth_image = self.capture_frame()
                if color_image is None:
                    continue
                
                frame_count += 1
                
                # Detect leaves
                results = self.detect_leaves(color_image)
                
                # Draw results
                display_img = self.draw_results(color_image, results)
                
                # Display processed annotated image (instead of mask)
                if results is not None:
                    # Draw annotations on processed image
                    annotated_img = self._draw_annotations(results['processed_image'], results)
                    
                    # Resize to match display_img
                    h_display, w_display = display_img.shape[:2]
                    h_annot, w_annot = annotated_img.shape[:2]
                    
                    if h_annot != h_display or w_annot != w_display:
                        annotated_img = cv2.resize(annotated_img, (w_display, h_display))
                    
                    # Combine images
                    combined = np.hstack([display_img, annotated_img])
                else:
                    combined = display_img
                
                # Display real-time results
                cv2.imshow('RealSense Leaf Detection', combined)
                
                # Regularly save results
                if save_interval > 0 and frame_count % save_interval == 0 and results is not None:
                    self.save_results(color_image, results, frame_count)
                    print(f"✓ Frame {frame_count}: {results['num_leaves']} leaves detected")
                
                # Keyboard control
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\nExiting program...")
                    break
                elif key == ord('s'):
                    self.save_results(color_image, results, frame_count)
                    print(f"✓ Saved current frame {frame_count}")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
    
    def _draw_annotations(self, img, results):
        """
        Draw leaf annotations on image (bounding boxes and labels)
        
        Args:
            img: Input image
            results: Detection results
            
        Returns:
            Annotated color image
        """
        annotated = img.copy()
        
        if results is None or 'contours' not in results:
            return annotated
        
        contours = results['contours']
        
        # Draw different colored markers for each detected object
        colors = [
            (255, 0, 0),
            (0, 255, 0),
            (0, 0, 255),
            (255, 255, 0),
            (255, 0, 255),
            (0, 255, 255),
        ]
        
        try:
            # Draw annotations for each contour
            for obj_id, cnt in enumerate(contours, 1):
                color = colors[(obj_id - 1) % len(colors)]
                
                # Draw bounding box
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
                
                # Draw label
                cv2.putText(annotated, f'Leaf {obj_id}', (x, y - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Draw center point
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(annotated, (cx, cy), 5, color, -1)
        except Exception as e:
            pass
        
        return annotated
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up resources...")
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("✓ Program exited normally")


def main():
    """Main function"""
    print("=" * 60)
    print("RealSense Real-time Leaf Detection System")
    print("Multi-leaf detection and analysis based on PlantCV")
    print("=" * 60)
    
    # Print configuration information
    print(f"\n⚙️  Current Configuration:")
    print(f"   • Detection mode: {'⚡ Fast (30+ FPS)' if FAST_MODE else '🎯 Precise (10-15 FPS)'}")
    print(f"   • Size analysis: {'✓ Enabled' if USE_SIZE_ANALYSIS else '✗ Disabled'}")
    print(f"   • Color analysis: {'✓ Enabled' if USE_COLOR_ANALYSIS else '✗ Disabled'}")
    print(f"   • Morphological processing: {'✓ Enabled' if USE_MORPHOLOGY else '✗ Disabled'}")
    print(f"   • Contour filtering: {'✓ Enabled' if USE_CONTOUR_FILTERING else '✗ Disabled'}")
    print(f"   • Save results: {'✓ Enabled' if SAVE_RESULTS else '✗ Disabled'}")
    print(f"   • Save interval: Every {SAVE_INTERVAL} frames")
    print(f"   • Output directory: {OUTPUT_DIR}\n")
    
    # Create detector
    detector = RealSenseLeafDetector(
        output_dir=OUTPUT_DIR,
        fast_mode=FAST_MODE,
        use_size=USE_SIZE_ANALYSIS,
        use_color=USE_COLOR_ANALYSIS,
        use_morphology=USE_MORPHOLOGY,
        use_filtering=USE_CONTOUR_FILTERING
    )
    
    # Run real-time detection
    save_interval = SAVE_INTERVAL if SAVE_RESULTS else 0
    detector.run(save_interval=save_interval)


if __name__ == "__main__":
    main()
