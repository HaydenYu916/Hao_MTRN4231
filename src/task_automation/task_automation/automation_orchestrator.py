#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Automation Task Orchestrator
Automated task management integrating leaf detection and robot arm control
"""

import rclpy
from rclpy.node import Node
from arm_msgs.srv import LeafDetectionSrv
from geometry_msgs.msg import Point
import subprocess
import time
import sys


class AutomationOrchestrator(Node):
    """Automation Task Orchestrator - Coordinates leaf detection and robot arm movement"""
    
    def __init__(self):
        super().__init__('automation_orchestrator')
        
        # Create leaf detection service client
        self.client = self.create_client(LeafDetectionSrv, 'leaf_detection_srv')
        
        # Configuration parameters
        self.declare_parameter('min_area', 0.0)  # Minimum leaf area
        self.declare_parameter('confidence', 0.0)  # Detection confidence
        self.declare_parameter('offset_z', 0.15)  # Z-axis offset (safety distance when approaching leaf)
        
        # Home position configuration
        self.declare_parameter('home_x', 0.25)  # Home X coordinate
        self.declare_parameter('home_y', 0.10)  # Home Y coordinate
        self.declare_parameter('home_z', 0.55)  # Home Z coordinate
        
        # Trash bin configuration
        self.declare_parameter('trash_x', 0.10)  # Trash bin X coordinate
        self.declare_parameter('trash_y', 0.50)  # Trash bin Y coordinate
        self.declare_parameter('trash_z', 0.20)  # Trash bin discard position (bin top 0.05m + 0.15m above)
        
        self.min_area = self.get_parameter('min_area').value
        self.confidence = self.get_parameter('confidence').value
        self.offset_z = self.get_parameter('offset_z').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_z = self.get_parameter('home_z').value
        self.trash_x = self.get_parameter('trash_x').value
        self.trash_y = self.get_parameter('trash_y').value
        self.trash_z = self.get_parameter('trash_z').value
        
        self.get_logger().info("Automation Task Orchestrator started")
        self.get_logger().info(f"Configuration: min_area={self.min_area}, confidence={self.confidence}, offset_z={self.offset_z}")
        self.get_logger().info(f"Home position: x={self.home_x}, y={self.home_y}, z={self.home_z}")
        self.get_logger().info(f"Trash bin position: x={self.trash_x}, y={self.trash_y}, z={self.trash_z}")
    
    def wait_for_service(self, timeout_sec=30.0):
        """Wait for leaf detection service to be available"""
        self.get_logger().info(f"Waiting for leaf detection service... (timeout: {timeout_sec}s)")
        
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error("❌ Leaf detection service unavailable!")
            self.get_logger().error("Please ensure the following service is running:")
            self.get_logger().error("  - ros2 launch detect_leaf_pkg leaf_detection_server.launch.py")
            return False
        
        self.get_logger().info("✓ Leaf detection service ready")
        return True
    
    def detect_leaves(self):
        """Call leaf detection service"""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Leaf detection service unavailable")
            return None
        
        try:
            request = LeafDetectionSrv.Request()
            request.command = "detect"
            request.min_area = self.min_area
            request.confidence = self.confidence
            
            self.get_logger().info(f"Sending detection request: min_area={self.min_area}, confidence={self.confidence}")
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.log_detection_results(response)
                    return response
                else:
                    self.get_logger().error(f"Detection failed: {response.message}")
                    return None
            else:
                self.get_logger().error("Detection service call timeout")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Detection service call exception: {str(e)}")
            return None
    
    def log_detection_results(self, response):
        """Log detection results"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("🌿 Leaf Detection Results")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Status: {response.message}")
        self.get_logger().info(f"Leaves detected: {response.num_leaves}")
        
        for i, point in enumerate(response.coordinates):
            self.get_logger().info(
                f"  Leaf {i+1}: X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m"
            )
        
        if response.debug_info:
            self.get_logger().info(f"Debug info: {response.debug_info}")
        
        self.get_logger().info("=" * 80 + "\n")
    
    def move_arm_to_pose(self, x, y, z, add_offset=True):
        """Move robot arm to specified position"""
        # If add_offset is True, add offset on Z-axis
        target_z = z + self.offset_z if add_offset else z
        
        self.get_logger().info(f"Moving arm to position: x={x:.3f}m, y={y:.3f}m, z={target_z:.3f}m")
        
        try:
            # Use ros2 command to launch move_arm_to_pose node
            cmd = [
                'ros2', 'launch', 'arm_manipulation', 'move_arm_to_pose_launch.py',
                f'x:={x}', f'y:={y}', f'z:={target_z}'
            ]
            
            self.get_logger().info(f"Executing command: {' '.join(cmd)}")
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60.0  # 60 second timeout
            )
            
            # Detailed output results
            if result.returncode == 0:
                self.get_logger().info("✓ Arm movement successful")
                # Show details only in verbose mode when successful
                if result.stdout and len(result.stdout.strip()) > 0:
                    self.get_logger().debug(f"Output:\n{result.stdout}")
                return True
            else:
                # Detailed error output when failed
                self.get_logger().error("❌❌❌ Arm movement failed ❌❌❌")
                self.get_logger().error(f"Return code: {result.returncode}")
                if result.stdout and len(result.stdout.strip()) > 0:
                    self.get_logger().error(f"Standard output:\n{result.stdout}")
                if result.stderr and len(result.stderr.strip()) > 0:
                    self.get_logger().error(f"Error output:\n{result.stderr}")
                else:
                    self.get_logger().error("(No detailed error information)")
                return False
                
        except subprocess.TimeoutExpired as e:
            self.get_logger().error("❌ Arm movement timeout (>60s)")
            self.get_logger().error(f"Timeout details: {str(e)}")
            return False
        except Exception as e:
            self.get_logger().error(f"❌ Arm movement exception: {str(e)}")
            self.get_logger().error(f"Exception type: {type(e).__name__}")
            import traceback
            self.get_logger().error(f"Exception traceback:\n{traceback.format_exc()}")
            return False
    
    def move_arm_to_trash(self):
        """Move robot arm to trash bin"""
        self.get_logger().info(f"Moving to trash bin: x={self.trash_x:.3f}m, y={self.trash_y:.3f}m, z={self.trash_z:.3f}m")
        return self.move_arm_to_pose(self.trash_x, self.trash_y, self.trash_z, add_offset=False)
    
    def move_arm_to_home(self):
        """Move robot arm to home position"""
        self.get_logger().info(f"Returning to home: x={self.home_x:.3f}m, y={self.home_y:.3f}m, z={self.home_z:.3f}m")
        return self.move_arm_to_pose(self.home_x, self.home_y, self.home_z, add_offset=False)
    
    def run_automation_loop(self):
        """Run automation task loop"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Starting automation task loop")
        self.get_logger().info("=" * 80)
        
        # Detect leaves
        response = self.detect_leaves()
        
        if response is None or response.num_leaves == 0:
            self.get_logger().warn("No leaves detected, task ended")
            return False
        
        # Iterate through all detected leaf positions
        success_count = 0
        for i, leaf_point in enumerate(response.coordinates):
            self.get_logger().info(f"\nProcessing leaf {i+1}/{response.num_leaves}...")
            
            # Move arm to leaf position
            if self.move_arm_to_pose(leaf_point.x, leaf_point.y, leaf_point.z):
                success_count += 1
                self.get_logger().info(f"✓ Leaf {i+1} picked up")
                
                # Move to trash bin
                self.get_logger().info(f"Moving to trash bin to discard leaf {i+1}...")
                if self.move_arm_to_trash():
                    self.get_logger().info(f"✓ Leaf {i+1} discarded")
                else:
                    self.get_logger().warn(f"⚠ Leaf {i+1} discard failed")
            else:
                self.get_logger().warn(f"⚠ Leaf {i+1} pickup failed")
            
            # Wait before processing next leaf
            if i < response.num_leaves - 1:
                wait_time = 2.0
                self.get_logger().info(f"Waiting {wait_time}s before processing next leaf...")
                time.sleep(wait_time)
        
        # Task completion summary
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Leaf processing complete")
        self.get_logger().info(f"Successfully processed: {success_count}/{response.num_leaves} leaves")
        self.get_logger().info("=" * 80 + "\n")
        
        # Return to home
        self.get_logger().info("Starting return to home...")
        home_success = self.move_arm_to_home()
        
        if home_success:
            self.get_logger().info("✓ Returned to home position")
        else:
            self.get_logger().error("\n" + "!" * 80)
            self.get_logger().error("❌❌❌ WARNING: Return to home failed! ❌❌❌")
            self.get_logger().error("!" * 80)
            self.get_logger().error(f"Target position: x={self.home_x:.3f}m, y={self.home_y:.3f}m, z={self.home_z:.3f}m")
            self.get_logger().error("Please check the actual arm position in RViz")
            self.get_logger().error("Manual arm movement or path re-planning may be needed")
            self.get_logger().error("!" * 80 + "\n")
        
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("Automation task flow complete")
        if not home_success:
            self.get_logger().info("⚠ Notice: Arm did not successfully return to home")
        self.get_logger().info("=" * 80 + "\n")
        
        return True  # Consider task flow complete even if return home failed


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = AutomationOrchestrator()
    
    # Wait for service availability
    if not orchestrator.wait_for_service(timeout_sec=30.0):
        orchestrator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    
    try:
        # Run automation task
        success = orchestrator.run_automation_loop()
        
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()
        print("Automation orchestrator shutdown complete")
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
