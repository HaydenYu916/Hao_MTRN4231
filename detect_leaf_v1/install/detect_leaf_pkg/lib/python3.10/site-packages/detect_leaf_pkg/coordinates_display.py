#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Leaf Coordinate Display Script
Subscribe to /leaf_detection/coordinates topic and display coordinate information in formatted way
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys

class CoordinatesDisplay(Node):
    def __init__(self):
        super().__init__('coordinates_display')
        
        self.subscription = self.create_subscription(
            String,
            '/leaf_detection/coordinates',
            self.callback,
            10
        )
        
        # Subscribe to transformed coordinates (base frame)
        self.base_subscription = self.create_subscription(
            String,
            '/leaf_detection/base_coordinates',
            self.base_callback,
            10
        )
        
        self.get_logger().info('Leaf coordinate display node started')
    
    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # Clear screen
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.flush()
            
            print("=" * 80)
            print("Leaf Detection Coordinates")
            print("=" * 80)
            print(f"Frame: {data.get('frame', 'N/A')}")
            print(f"Timestamp: {data.get('timestamp', 'N/A')}")
            print(f"Detected leaves: {data.get('num_leaves', 0)}")
            
            # Display camera intrinsics
            camera_params = data.get('camera_params', None)
            if camera_params:
                print("-" * 80)
                print("📷 Camera Intrinsics:")
                print(f"   Resolution: {camera_params.get('width', 0)}x{camera_params.get('height', 0)}")
                print(f"   Focal: fx={camera_params.get('fx', 0):.2f}, fy={camera_params.get('fy', 0):.2f}")
                print(f"   Principal: cx={camera_params.get('ppx', 0):.2f}, cy={camera_params.get('ppy', 0):.2f}")
            
            print("-" * 80)
            
            coordinates = data.get('coordinates', [])
            for leaf in coordinates:
                leaf_id = leaf.get('id', 'N/A')
                center = leaf.get('center', {})
                bbox = leaf.get('bounding_box', {})
                area = leaf.get('area', 0)
                perimeter = leaf.get('perimeter', 0)
                depth_mm = leaf.get('depth_mm', 0)
                point_3d = leaf.get('point_3d', None)
                
                print(f"\n📍 Leaf #{leaf_id}")
                print(f"   Pixel: ({center.get('x', 0)}, {center.get('y', 0)})")
                print(f"   BBox: x={bbox.get('x', 0)}, y={bbox.get('y', 0)}, "
                      f"w={bbox.get('width', 0)}, h={bbox.get('height', 0)}")
                print(f"   Area: {area:.2f} px²")
                print(f"   Perimeter: {perimeter:.2f} px")
                
                # Depth info
                if depth_mm > 0:
                    depth_m = depth_mm * 0.001
                    print(f"   Depth: {depth_mm:.2f} mm ({depth_m:.3f} m)")
                else:
                    print(f"   Depth: N/A")
                
                # 3D coordinates
                if point_3d is not None:
                    print(f"   3D: X={point_3d[0]:.3f}m, Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m")
                else:
                    print(f"   3D: N/A")
            
            print("\n" + "=" * 80)
            
        except Exception as e:
            self.get_logger().error(f'✗ Display coordinates error: {str(e)}')
    
    def base_callback(self, msg):
        """处理转换后的基coordinates信息"""
        try:
            data = json.loads(msg.data)
            
            # Clear screen
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.flush()
            
            print("=" * 80)
            print("🎯 Leaf Detection Coordinates(Robot Base Frame)")
            print("=" * 80)
            print(f"Frame: {data.get('frame', 'N/A')}")
            print(f"Timestamp: {data.get('timestamp', 'N/A')}")
            print(f"Detected leaves: {data.get('num_leaves', 0)}")
            
            # Display camera intrinsics
            camera_params = data.get('camera_params', None)
            if camera_params:
                print("-" * 80)
                print("📷 Camera Intrinsics:")
                print(f"   Resolution: {camera_params.get('width', 0)}x{camera_params.get('height', 0)}")
                print(f"   Focal: fx={camera_params.get('fx', 0):.2f}, fy={camera_params.get('fy', 0):.2f}")
                print(f"   Principal: cx={camera_params.get('ppx', 0):.2f}, cy={camera_params.get('ppy', 0):.2f}")
            
            print("-" * 80)
            
            coordinates = data.get('coordinates', [])
            for leaf in coordinates:
                leaf_id = leaf.get('id', 'N/A')
                center = leaf.get('center', {})
                bbox = leaf.get('bounding_box', {})
                area = leaf.get('area', 0)
                perimeter = leaf.get('perimeter', 0)
                depth_mm = leaf.get('depth_mm', 0)
                point_3d = leaf.get('point_3d', None)
                base_coords = leaf.get('base_coordinates', None)
                
                print(f"\n📍 Leaf #{leaf_id}")
                print(f"   Pixel: ({center.get('x', 0)}, {center.get('y', 0)})")
                print(f"   BBox: x={bbox.get('x', 0)}, y={bbox.get('y', 0)}, "
                      f"w={bbox.get('width', 0)}, h={bbox.get('height', 0)}")
                print(f"   Area: {area:.2f} px²")
                print(f"   Perimeter: {perimeter:.2f} px")
                
                # Depth info
                if depth_mm > 0:
                    depth_m = depth_mm * 0.001
                    print(f"   Depth: {depth_mm:.2f} mm ({depth_m:.3f} m)")
                else:
                    print(f"   Depth: N/A")
                
                # 3D coordinates in camera frame
                if point_3d is not None:
                    print(f"   Camera: X={point_3d[0]:.3f}m, Y={point_3d[1]:.3f}m, Z={point_3d[2]:.3f}m")
                else:
                    print(f"   Camera: N/A")
                
                # 3D coordinates in base frame
                if base_coords is not None:
                    print(f"   🎯 Base: X={base_coords[0]:.3f}m, Y={base_coords[1]:.3f}m, Z={base_coords[2]:.3f}m")
                else:
                    print(f"   🎯 Base: N/A")
            
            print("\n" + "=" * 80)
            
        except Exception as e:
            self.get_logger().error(f'✗ Display base coordinates error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CoordinatesDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✓ Coordinate display node stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
