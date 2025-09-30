from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )
    

    map_file = os.path.join(
        get_package_share_directory("nav_pkg"), 
        "maps", 
        "map_modified2.yaml"
    )
    nav2_params_file = os.path.join(
        get_package_share_directory("nav_pkg"), 
        "config", 
        "nav2_params.yaml"
    )
   
    rviz_config_file = os.path.join(
        get_package_share_directory("nav2_bringup"), 
        "rviz", 
        "nav2_default_view.rviz"  
    )
    
 
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "True",
            "params_file": nav2_params_file
        }.items()
    )
    

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],  
        output="screen"
    )
    
   
    # waypoint_client = Node(
    #     package="nav_pkg",
    #     executable="nav2_waypoint_client",
    #     output="screen",
    #     parameters=[{"use_sim_time": True}]
    # )
    
   
    

    return LaunchDescription([
        nav2_launch,
        rviz_node, 
    ])