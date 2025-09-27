import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pub_cmd = Node(
        package="status_publisher",
        executable="sys_status_pub",
        output="screen"
    )

    display_cmd= Node(
        package="status_display",
        executable="status_display",
        output="screen"
    )

    ld = LaunchDescription()
    ld.add_action(pub_cmd)
    ld.add_action(display_cmd)

    return ld