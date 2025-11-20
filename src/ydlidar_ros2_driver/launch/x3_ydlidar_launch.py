#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # 获取包的共享目录（确保路径正确）
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    
    # 声明参数文件路径（显式指定 X3 的参数文件）
    params_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_x3.yaml'),
        description='Path to the ROS2 parameters file (ydlidar_x3.yaml)'
    )

    # 激光雷达节点（使用 LifecycleNode，显式指定名称和命名空间）
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',  # 确保可执行文件名称正确
        name='ydlidar_ros2_driver_node',        # 必须：节点名称（与代码中一致）
        namespace='',                           # 简化命名空间（避免路径问题）
        output='screen',
        emulate_tty=True,
        parameters=[params_file]                # 加载参数文件
    )

    # TF 转换节点（可选，根据需要启用）
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    return LaunchDescription([
        params_declare,
        LogInfo(msg=f"Using parameter file: {params_file}"),  # 打印参数文件路径，便于调试
        driver_node,
        tf2_node  # 如果需要激光雷达到基座的坐标转换，取消注释
    ])