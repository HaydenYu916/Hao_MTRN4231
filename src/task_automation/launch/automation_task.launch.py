#!/usr/bin/env python3
"""
自动化任务启动文件
启动自动化编排器节点
"""

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='0.0',
        description='叶子检测最小面积阈值'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.0',
        description='叶子检测置信度阈值'
    )
    
    offset_z_arg = DeclareLaunchArgument(
        'offset_z',
        default_value='0.05',
        description='机械臂Z轴偏移量（米）'
    )
    
    home_x_arg = DeclareLaunchArgument(
        'home_x',
        default_value='0.25',
        description='原位X坐标（米）'
    )
    
    home_y_arg = DeclareLaunchArgument(
        'home_y',
        default_value='0.10',
        description='原位Y坐标（米）'
    )
    
    home_z_arg = DeclareLaunchArgument(
        'home_z',
        default_value='0.55',
        description='原位Z坐标（米）'
    )
    
    trash_x_arg = DeclareLaunchArgument(
        'trash_x',
        default_value='0.10',
        description='垃圾桶X坐标（米）'
    )
    
    trash_y_arg = DeclareLaunchArgument(
        'trash_y',
        default_value='0.50',
        description='垃圾桶Y坐标（米）'
    )
    
    trash_z_arg = DeclareLaunchArgument(
        'trash_z',
        default_value='0.20',
        description='垃圾桶倾倒位置Z坐标（米）'
    )
    
    # 创建自动化编排器节点
    orchestrator_node = Node(
        package='task_automation',
        executable='automation_orchestrator',
        name='automation_orchestrator',
        output='screen',
        parameters=[{
            'min_area': ParameterValue(LaunchConfiguration('min_area'), value_type=float),
            'confidence': ParameterValue(LaunchConfiguration('confidence'), value_type=float),
            'offset_z': ParameterValue(LaunchConfiguration('offset_z'), value_type=float),
            'home_x': ParameterValue(LaunchConfiguration('home_x'), value_type=float),
            'home_y': ParameterValue(LaunchConfiguration('home_y'), value_type=float),
            'home_z': ParameterValue(LaunchConfiguration('home_z'), value_type=float),
            'trash_x': ParameterValue(LaunchConfiguration('trash_x'), value_type=float),
            'trash_y': ParameterValue(LaunchConfiguration('trash_y'), value_type=float),
            'trash_z': ParameterValue(LaunchConfiguration('trash_z'), value_type=float),
        }],
    )
    
    # 启动信息
    info_msg = LogInfo(
        msg='自动化任务编排器已启动'
    )
    
    return launch.LaunchDescription([
        min_area_arg,
        confidence_arg,
        offset_z_arg,
        home_x_arg,
        home_y_arg,
        home_z_arg,
        trash_x_arg,
        trash_y_arg,
        trash_z_arg,
        info_msg,
        orchestrator_node,
    ])

