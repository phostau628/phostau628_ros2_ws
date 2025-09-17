from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction  # 导入TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 嵌套启动第一个Launch文件（nav_launch.py）
    nav_pkg_share = get_package_share_directory("nav_pkg")
    nav_launch_path = os.path.join(nav_pkg_share, "launch", "nav_launch.py")
    include_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path)
    )

    # 2. 嵌套启动第二个Launch文件（robocup_home.launch.py）
    wpr_sim_share = get_package_share_directory("wpr_simulation2")
    robocup_launch_path = os.path.join(wpr_sim_share, "launch", "robocup_home.launch.py")
    include_robocup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robocup_launch_path)
    )

    # 3. 定义需要延时启动的节点（waypoint_navigation）
    waypoint_nav_node = Node(
        package="nav_pkg",
        executable="waypoint_navigation",
        output="screen"
    )

    # 4. 用TimerAction包装节点，设置延时10秒
    delayed_node = TimerAction(
        period=30.0,  # 延时时间（秒）
        actions=[waypoint_nav_node]  # 延时后执行的动作（启动节点）
    )

    # 组装：先启动两个Launch文件，再延时启动节点
    return LaunchDescription([
        include_nav_launch,
        include_robocup_launch,
        delayed_node  # 延时10秒后启动节点
    ])