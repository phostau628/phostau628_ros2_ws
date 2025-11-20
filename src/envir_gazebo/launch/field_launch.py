from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取功能包路径
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    envir_gazebo_share = get_package_share_directory('envir_gazebo')
    
    # 配置 Gazebo 插件路径（确保能找到 ROS 插件）
    gazebo_plugin_path = os.path.join(os.environ['HOME'], 'ros2_ws', 'install', 'envir_gazebo', 'lib')
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        gazebo_plugin_path += ':' + os.environ['GAZEBO_PLUGIN_PATH']
    
    # 显式指定空世界文件路径（关键修复）
   # 替换原来的 empty_world_path 定义，用 ROS2 自带的空世界
    empty_world_path = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')
    
    # 启动 Gazebo 服务端和客户端，强制加载 ROS 工厂插件
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': empty_world_path,
            'server_required_plugins': 'libgazebo_ros_factory.so',  # 强制加载工厂插件
            'verbose': 'true'
        }.items()
    )
    
    # 插入模型
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(envir_gazebo_share, 'models', 'field_model', 'model.sdf'),
            '-entity', 'field_model',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        start_gazebo,
        spawn_model
    ])