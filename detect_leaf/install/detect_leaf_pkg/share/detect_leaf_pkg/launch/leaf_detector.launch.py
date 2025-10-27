#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    生成启动描述
    """
    
    # 声明启动参数
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='输入图像话题名称'
    )
    
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='/leaf_detection_result',
        description='检测结果输出话题名称'
    )
    
    vector_topic_arg = DeclareLaunchArgument(
        'vector_topic',
        default_value='/leaf_center_vector',
        description='叶子中心向量输出话题名称'
    )
    
    # 叶子检测节点
    leaf_detector_node = Node(
        package='detect_leaf_pkg',
        executable='leaf_detector',
        name='leaf_detector',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'detection_topic': LaunchConfiguration('detection_topic'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        image_topic_arg,
        detection_topic_arg,
        vector_topic_arg,
        leaf_detector_node,
    ])
