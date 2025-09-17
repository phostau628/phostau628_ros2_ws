# 1. 导入Python内置的os模块：用于处理文件路径（如拼接、获取目录），ROS 2 Launch中常用于定位配置/地图/RViz文件
import os

# 2. 导入Launch核心类LaunchDescription：
#    - 所有Launch文件的"根容器"，用于承载所有要启动的节点（Node）、参数（Parameter）、动作（Action）等
#    - 最终通过generate_launch_description()函数返回，告诉ROS 2要启动哪些资源
from launch import LaunchDescription

# 3. 导入Launch-ROS的Node类：
#    - 用于在Launch文件中定义一个ROS 2节点（可理解为"要启动的程序"）
#    - 需指定节点所属包（package）、可执行文件名（executable）、参数（parameters）等关键信息
from launch_ros.actions import Node

# 4. 导入ament_index_python的get_package_share_directory函数：
#    - ROS 2的核心工具函数，用于获取某个ROS 2包的"共享目录路径"（即install/包名/share/包名/）
#    - 解决硬编码路径问题（如直接写"/home/user/xxx"），确保在不同环境下都能找到包内资源（如RViz配置、参数文件）
from ament_index_python.packages import get_package_share_directory

# 5. 导入ROS 2的QoS配置相关类：
#    - QoS（Quality of Service，服务质量）用于定义ROS 2话题通信的可靠性、历史记录、持久性等策略
#    - 此处虽未直接使用，但导入后可用于后续扩展（如给节点配置自定义QoS，避免话题通信丢包/延迟）
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

# 6. 定义Launch文件的核心函数：generate_launch_description()
#    - 固定函数名，ROS 2 Launch框架会自动调用该函数获取要启动的资源
#    - 函数内需构建并返回一个LaunchDescription对象
def generate_launch_description():

    # 7. 定义slam_toolbox节点的参数字典：slam_params
    #    - 字典的key是参数名（需与slam_toolbox节点的参数定义一致），value是参数值
    #    - 作用：集中管理节点参数，避免在Node()中零散写参数，便于后续修改和维护
    slam_params={
        "use_sim_time":True,  # 是否使用仿真时间（True：用 Gazebo 等仿真器的时间；False：用真实硬件的系统时间）
        "base_frame":"base_footprint",  # 机器人基坐标系（通常是机器人底盘中心的坐标系，与轮子运动关联）
        "odom_frame":"odom",  # 里程计坐标系（用于记录机器人相对初始位置的运动，易累积误差）
        "map_frame":"map",    # 地图坐标系（全局固定坐标系，SLAM构建的地图基于此坐标系）
        "min_pass_through": 1,         # 自由空间判定阈值：只需1束激光未命中障碍物，就标记该网格为"自由空间"（原3，改小后更灵敏）
        "occupancy_threshold": 0.2,    # 占据阈值：网格占据概率超过0.2时，标记为"障碍物"（值越小，越容易判定为障碍物）
        "map_update_interval": 1.0,    # 地图更新间隔：每1秒更新一次全局地图（平衡实时性和计算量）
        "minimum_time_interval": 0.1,  # 激光数据处理间隔：每0.1秒处理一次新激光扫描（原0.3，改小后更频繁更新）
        "throttle_scans": 1,           # 激光数据节流：每接收1帧激光就处理（1=不节流，>1则每隔N帧处理一次，用于降低计算量）
        "resolution": 0.08,            # 地图分辨率：0.08米/像素（即每个网格边长0.08米，值越小地图越精细但占用内存越大）
        "min_laser_range": 0.1,        # 激光最小有效距离：过滤0.1米内的激光点（避免机器人自身结构遮挡产生的噪点）
        "max_laser_range": 8.0,        # 激光最大有效距离：只处理8米内的激光点（超出范围的点视为无效，避免远距离噪声干扰）
        "stack_size_to_use": 30000000, # 节点栈大小：30MB（给SLAM算法分配的内存栈，避免复杂计算时栈溢出）
        "do_loop_closing": True,       # 是否开启闭环检测（核心功能：SLAM过程中检测机器人是否回到曾到过的位置，修正累积误差）
        "loop_search_maximum_distance": 4.0,      # 闭环搜索最大距离：只在4米范围内搜索可能的闭环点（室内场景适配，避免无效搜索）
        "loop_match_minimum_chain_size": 8,       # 最小闭环链长度：机器人至少移动8个关键帧后才开始闭环检测（原10，改小后更早触发闭环）
        "loop_match_maximum_variance_coarse": 2.0, # 粗匹配方差阈值：粗匹配时位置方差超过2.0则过滤（缩小阈值，减少错误闭环）
        "loop_match_minimum_response_coarse": 0.4, # 粗匹配响应阈值：粗匹配得分低于0.4则视为无效（过滤低可信度的匹配，减少计算）
        "loop_match_minimum_response_fine": 0.5,   # 精匹配响应阈值：精匹配得分低于0.5则视为无效（进一步确保闭环匹配的准确性）
        "use_scan_matching": True,     # 是否启用扫描匹配（核心功能：将当前激光帧与局部地图匹配，优化机器人位姿）
        "use_scan_barycenter": True,   # 是否启用扫描重心：以激光点云的重心为基准进行匹配（提高匹配稳定性，减少局部极值干扰）
        "minimum_travel_distance": 0.5, # 位姿更新最小移动距离：机器人至少移动0.5米才更新位姿（避免小位移时频繁计算，节省资源）
        "minimum_travel_heading": 0.5  # 位姿更新最小转向角度：机器人至少转向0.5弧度（约28.6度）才更新位姿（避免小角度转向时频繁计算）
        # 注：此处原代码有一行注释"# 3束激光命中才判定占据..."，属于注释残留，不影响功能
    }

    # 8. 创建slam_toolbox的节点实例：slam_cmd
    #    - Node()是launch_ros.actions中的核心类，代表一个要启动的ROS 2节点
    slam_cmd = Node(
        package="slam_toolbox",          # 节点所属的ROS 2包名：slam_toolbox（SLAM算法包）
        executable="sync_slam_toolbox_node", # 节点的可执行文件名：同步模式的SLAM节点（适合激光频率与处理速度匹配的场景）
        parameters=[slam_params]         # 传递给节点的参数：使用前面定义的slam_params字典（也可直接写{"key":value}）
        # 可选扩展参数：name="custom_slam_node"（自定义节点名，避免同名冲突）、output="screen"（将节点日志打印到终端）
    )

    # 9. 构建RViz配置文件的完整路径：rviz_file
    #    - 步骤1：用get_package_share_directory获取"wpr_simulation2"包的共享目录（如install/wpr_simulation2/share/wpr_simulation2/）
    #    - 步骤2：用os.path.join拼接目录与文件路径，得到完整的RViz配置文件路径（避免跨系统路径分隔符问题，如Windows用\，Linux用/）
    rviz_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),  # 第一个参数：RViz配置文件所在的包
        'rviz',                                          # 第二个参数：包内存储RViz文件的目录名
        'slam.rviz'                                      # 第三个参数：具体的RViz配置文件名（保存了SLAM所需的显示项，如地图、激光、机器人模型）
    )

    # 10. 创建RViz2的节点实例：rviz_cmd
    rviz_cmd=Node(
        package='rviz2',       # 节点所属包名：rviz2（ROS 2的可视化工具包）
        executable='rviz2',    # 可执行文件名：rviz2（RViz的主程序）
        name='rviz2',          # 自定义节点名：避免与其他RViz节点冲突（可选，默认是"rviz2"）
        arguments=['-d', rviz_file]  # 启动参数：-d 表示"使用指定的配置文件"，后面跟步骤9构建的rviz_file路径
    )
    
    # 11. 创建LaunchDescription实例：ld
    #    - 这是Launch文件的"容器"，所有要启动的节点（slam_cmd、rviz_cmd）都需要添加到这个容器中
    ld=LaunchDescription()

    # 12. 将SLAM节点添加到Launch容器中
    #    - add_action()是LaunchDescription的方法，用于添加"动作"（节点、参数、条件等）
    ld.add_action(slam_cmd)

    # 13. 将RViz节点添加到Launch容器中
    #    - 添加后，启动Launch文件时会同时启动slam_toolbox和rviz2两个节点
    ld.add_action(rviz_cmd)

    # 14. 返回Launch容器：ROS 2 Launch框架会根据该容器中的内容启动所有资源
    return ld


'''
Toolbox Params
odom_frame - Odometry frame

map_frame - Map frame

base_frame - Base frame

scan_topic - scan topic, absolute path, i.e. /scan not scan

restamp_tf - Whether to restamp the TF messages with the current time or use the scan's message. Default False.

scan_queue_size - The number of scan messages to queue up before throwing away old ones. Should always be set to 1 in async mode

use_map_saver - Instantiate the map saver service and self-subscribe to the map topic

map_file_name - Name of the pose-graph file to load on startup if available

map_start_pose - Pose to start pose-graph mapping/localization in, if available

map_start_at_dock - Starting pose-graph loading at the dock (first node), if available. If both pose and dock are set, it will use pose

debug_logging - Change logger to debug

throttle_scans - Number of scans to throttle in synchronous mode

transform_publish_period - The map to odom transform publish period. 0 will not publish transforms

map_update_interval - Interval to update the 2D occupancy map for other applications / visualization

enable_interactive_mode - Whether or not to allow for interactive mode to be enabled. Interactive mode will retain a cache of laser scans mapped to their ID for visualization in interactive mode. As a result the memory for the process will increase. This is manually disabled in localization and lifelong modes since they would increase the memory utilization over time. Valid for either mapping or continued mapping modes.

position_covariance_scale - Amount to scale position covariance when publishing pose from scan match. This can be used to tune the influence of the pose position in a downstream localization filter. The covariance represents the uncertainty of the measurement, so scaling up the covariance will result in the pose position having less influence on downstream filters. Default: 1.0

yaw_covariance_scale - Amount to scale yaw covariance when publishing pose from scan match. See description of position_covariance_scale. Default: 1.0

resolution - Resolution of the 2D occupancy map to generate

min_laser_range - Minimum laser range to use for 2D occupancy map rasterizing

max_laser_range - Maximum laser range to use for 2D occupancy map rasterizing

minimum_time_interval - The minimum duration of time between scans to be processed in synchronous mode

transform_timeout - TF timeout for looking up transforms

tf_buffer_duration - Duration to store TF messages for lookup. Set high if running offline at multiple times speed in synchronous mode.

stack_size_to_use - The number of bytes to reset the stack size to, to enable serialization/deserialization of files. A liberal default is 40000000, but less is fine.

minimum_travel_distance - Minimum distance of travel before processing a new scan

localization_on_configure - Set to true to set the localization mode to localization during node on_configure transition. Set to false to set the localization mode to mapping instead. Only applies to map_and_localization_slam_toolbox node.

Matcher Params
use_scan_matching - whether to use scan matching to refine odometric pose (uh, why would you not?)

use_scan_barycenter - Whether to use the barycenter or scan pose

minimum_travel_heading - Minimum changing in heading to justify an update

scan_buffer_size - The number of scans to buffer into a chain, also used as the number of scans in the circular buffer of localization mode

scan_buffer_maximum_scan_distance - Maximum distance of a scan from the pose before removing the scan from the buffer

link_match_minimum_response_fine - The threshold link matching algorithm response for fine resolution to pass

link_scan_maximum_distance - Maximum distance between linked scans to be valid

loop_search_maximum_distance - Maximum threshold of distance for scans to be considered for loop closure

do_loop_closing - Whether to do loop closure (if you're not sure, the answer is "true")

loop_match_minimum_chain_size - The minimum chain length of scans to look for loop closure

loop_match_maximum_variance_coarse - The threshold variance in coarse search to pass to refine

loop_match_minimum_response_coarse - The threshold response of the loop closure algorithm in coarse search to pass to refine

loop_match_minimum_response_fine - The threshold response of the loop closure algorithm in fine search to pass to refine

correlation_search_space_dimension - Search grid size to do scan correlation over

correlation_search_space_resolution - Search grid resolution to do scan correlation over

correlation_search_space_smear_deviation - Amount of multimodal smearing to smooth out responses

loop_search_space_dimension - Size of the search grid over the loop closure algorithm

loop_search_space_resolution - Search grid resolution to do loop closure over

loop_search_space_smear_deviation - Amount of multimodal smearing to smooth out responses

distance_variance_penalty - A penalty to apply to a matched scan as it differs from the odometric pose

angle_variance_penalty - A penalty to apply to a matched scan as it differs from the odometric pose

fine_search_angle_offset - Range of angles to test for fine scan matching

coarse_search_angle_offset - Range of angles to test for coarse scan matching

coarse_angle_resolution - Resolution of angles over the Offset range to test in scan matching

minimum_angle_penalty - Smallest penalty an angle can have to ensure the size doesn't blow up

minimum_distance_penalty - Smallest penalty a scan can have to ensure the size doesn't blow up

use_response_expansion - Whether to automatically increase the search grid size if no viable match is found

min_pass_through - Number of beams that must pass through a cell before it will be considered to be occupied or unoccupied. This prevents stray beams from messing up the map.

occupancy_threshold - Minimum ratio of beams hitting cell to beams passing through cell to be marked as occupied'''