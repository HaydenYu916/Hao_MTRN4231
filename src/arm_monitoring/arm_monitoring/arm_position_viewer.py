#!/usr/bin/env python3
"""
View UR5e robot arm position and coordinate information
"""
import os
import sys
import ctypes

# Fix Conda and ROS2 library conflicts: prioritize system libraries
# Must be set before importing any ROS2 libraries
if 'CONDA_PREFIX' in os.environ:
    conda_prefix = os.environ['CONDA_PREFIX']
    system_lib_path = '/usr/lib/x86_64-linux-gnu'
    
    # Set environment variables
    if 'LD_LIBRARY_PATH' in os.environ:
        current_path = os.environ['LD_LIBRARY_PATH']
        new_path = f'{system_lib_path}:{conda_prefix}/lib:{current_path}'
    else:
        new_path = f'{system_lib_path}:{conda_prefix}/lib'
    
    os.environ['LD_LIBRARY_PATH'] = new_path
    os.putenv('LD_LIBRARY_PATH', new_path)
    
    # Preload system libstdc++ to ensure priority
    try:
        lib_path = os.path.join(system_lib_path, 'libstdc++.so.6')
        if os.path.exists(lib_path):
            ctypes.CDLL(lib_path, mode=ctypes.RTLD_GLOBAL)
    except Exception:
        pass  # If preload fails, continue with environment variable settings

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import math

class ArmPositionViewer(Node):
    def __init__(self):
        super().__init__('arm_position_viewer')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_data = None
        self.get_logger().info('Arm position viewer started!')
        self.get_logger().info('=' * 60)
        
        # Timer: display information every 2 seconds
        self.timer = self.create_timer(2.0, self.display_info)
    
    def joint_callback(self, msg):
        """Receive joint state"""
        self.joint_data = msg
    
    def display_info(self):
        """Display robot arm position information"""
        if self.joint_data is None:
            self.get_logger().warn('Waiting for joint data...')
            return
        
        print("\n" + "=" * 60)
        print("📍 Current Robot Arm Status")
        print("=" * 60)
        
        # 1. Display joint angles
        print("\n🔧 Joint Angles (radians / degrees):")
        for i, name in enumerate(self.joint_data.name):
            pos_rad = self.joint_data.position[i]
            pos_deg = math.degrees(pos_rad)
            print(f"  {name:25s}: {pos_rad:8.4f} rad ({pos_deg:7.2f}°)")
        
        # 2. Display end effector position (from base_link to tool0)
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            print("\n📐 End Effector Position (base_link → tool0):")
            print(f"  Position (x, y, z):")
            print(f"    x: {transform.transform.translation.x:8.4f} m")
            print(f"    y: {transform.transform.translation.y:8.4f} m")
            print(f"    z: {transform.transform.translation.z:8.4f} m")
            
            print(f"  Orientation (quaternion):")
            print(f"    x: {transform.transform.rotation.x:8.4f}")
            print(f"    y: {transform.transform.rotation.y:8.4f}")
            print(f"    z: {transform.transform.rotation.z:8.4f}")
            print(f"    w: {transform.transform.rotation.w:8.4f}")
            
            # Convert to Euler angles (RPY)
            q = transform.transform.rotation
            roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
            print(f"  Orientation (Euler angles RPY):")
            print(f"    Roll:  {math.degrees(roll):7.2f}°")
            print(f"    Pitch: {math.degrees(pitch):7.2f}°")
            print(f"    Yaw:   {math.degrees(yaw):7.2f}°")
            
        except Exception as e:
            self.get_logger().warn(f'Failed to get TF transform: {str(e)}')
        
        # 3. Display other coordinate frames
        try:
            frames = ['base_link', 'shoulder_link', 'wrist_3_link', 'tool0']
            print(f"\n🌐 Available coordinate frames: {', '.join(frames)}")
        except Exception as e:
            pass
        
        print("=" * 60)
    
    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    viewer = ArmPositionViewer()
    
    print("\n🤖 UR5e Robot Arm Position Viewer")
    print("Press Ctrl+C to exit\n")
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\n\nProgram exited")
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




