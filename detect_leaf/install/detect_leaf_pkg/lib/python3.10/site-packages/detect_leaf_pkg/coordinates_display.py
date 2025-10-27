#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
叶子坐标信息显示脚本
订阅 /leaf_detection/coordinates 话题并以格式化方式显示坐标信息
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
        
        self.get_logger().info('🌿 坐标信息显示节点已启动')
    
    def callback(self, msg):
        try:
            data = json.loads(msg.data)
            
            # 清屏
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.flush()
            
            print("=" * 80)
            print("🌿 叶子检测坐标信息")
            print("=" * 80)
            print(f"帧号: {data.get('frame', 'N/A')}")
            print(f"时间戳: {data.get('timestamp', 'N/A')}")
            print(f"检测到的叶子数: {data.get('num_leaves', 0)}")
            print("-" * 80)
            
            coordinates = data.get('coordinates', [])
            for leaf in coordinates:
                leaf_id = leaf.get('id', 'N/A')
                center = leaf.get('center', {})
                bbox = leaf.get('bounding_box', {})
                area = leaf.get('area', 0)
                perimeter = leaf.get('perimeter', 0)
                
                print(f"\n📍 叶子 #{leaf_id}")
                print(f"   中心点: ({center.get('x', 0)}, {center.get('y', 0)})")
                print(f"   边界框: x={bbox.get('x', 0)}, y={bbox.get('y', 0)}, "
                      f"w={bbox.get('width', 0)}, h={bbox.get('height', 0)}")
                print(f"   面积: {area:.2f} 像素²")
                print(f"   周长: {perimeter:.2f} 像素")
            
            print("\n" + "=" * 80)
            
        except Exception as e:
            self.get_logger().error(f'✗ 显示坐标错误: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CoordinatesDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n✓ 坐标显示节点已停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
