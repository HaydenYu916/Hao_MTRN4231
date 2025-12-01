from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ⚠️ These are verified calibration parameters, ensure you understand them before modifying
    # Position (meters)
    # x, y, z = '1.30317', '0.0174152', '0.675776' G2
    x, y, z = '1.27354', '0.0326318', '0.68138' # G6
    # Quaternion (verified values, corresponding to RPY)
    # RPY: roll=0.0372615, pitch=-0.815369, yaw=-3.12141
        # 1.29163 0.0146956 0.665489
    # qx, qy, qz, qw = '-0.388123', '-0.0054127', '0.92155', '-0.0087602' G2
    qx, qy, qz, qw = '-0.397486', '-0.00834818', '0.91757', '-0.000592307' #G6
    return LaunchDescription([
        # Transform from base_link to camera_link (hand-eye calibration)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_broadcaster',
            arguments=[x, y, z, qx, qy, qz, qw, 'base_link', 'camera_link']
        ),
        # Transform from camera_link to camera_color_optical_frame
        # RealSense standard optical frame rotation
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_frame_broadcaster',
            arguments=['0', '0', '0', '0.5', '-0.5', '0.5', '0.5',
                      'camera_link', 'camera_color_optical_frame']
        )
    ])
