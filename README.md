# 导航功能包说明文档

## turtle_pkg：小海龟"RM"轨迹实现

该包通过节点间的话题发布与接收，控制小海龟运动走出"RM"形状，并考虑实体车特点加入闭环控制思想。实现逻辑如下：

### 1. 创建类
创建类TurtleRMNode，继承自rclcpp::Node

### 2. 节点初始化
首先创建名为"turtle_rm"的节点，随后初始化发布者、订阅者，并设置定时器（与核心控制回调函数TimerCallback绑定）和设置小乌龟需经过的八个坐标，最后初始化索引"current_target_idx=0"，并打印启动日志。

### 3. 获取小乌龟位姿信息
通过poseCallback函数，接收/turtle/pose的信息，并将其解引用，把x，y坐标存到current_pose，如此便可实时获取小乌龟的位姿信息。

### 4. 核心控制
(1) calculateDistance函数：计算当前位置和目标点距离，并存于变量distance中

(2) calculateTargetAngle函数：计算对准目标点时的角度

(3) calculateAngleError函数：计算当前角度和目标角度的差值（并将角度限制在-180 – 180之间）

(4) 转向直行的控制: 以角度差0.06为界，决定小龟执行转向还是直行

(5) 判断目标点是否到达: 设置距离误差为0.02，若distance小于等于0.02，则认为已到达，并将索引current_target_idx加1

(6) 判断任务是否完成: 以索引是否大于等于8为依据，若是，则发送停止指令，并用'return;' 跳过函数后续步骤，顺便打印日志 "PhosTau 滴任务完成了！！！"

### 5. 关键思想：闭环控制
(1) 反馈：通过订阅/turtle/pose话题，持续获取小海龟当前位姿，实现实时反馈

(2) 动态修正：使用std::min及std::max修正小海龟线速度,前者考虑实体车速度不可能像仿真一样实现突变，故需将尾段速度平滑处理，保障了安全性;后者则是为了保障效率，让小车在终点时不至于完全停住，保留微小速度。


## nav_pkg：可循环多点导航实现（waypoint_navigation）

该代码用于控制机器人实现多点导航，实现逻辑如下：

### 节点初始化
1. 创建类TurtleRMNode，继承自rclcpp::Node
2. 创建名为"waypoint_navigation"的节点
3. 初始化发布者，订阅者，并创建定时器，其与NavPub绑定

### 导航所需变量
- first_pub: 表示首次发布，即一号航点
- current_waypoint: 当前目标航点，从1开始，最大增到9,全部完成之后又回到1，如此循环。
- new_waypoint_needed: 用于控制是否发布新航点
- max_point: 顾名思义，最大航点数，即9.

### 发布航点NavPub
首次运行（first_pub)等待1s，然后根据new_waypoint_need以及max_point决定是否发布新航点，发布成功便生成日志，最后将new_waypoint_needed设成false，避免多余重复发布

### 导航反馈函数NavCallback
1. 监听导航结果，当收到 "navi done" 消息时，表示已到达当前航点
2. 若当前航点小于max_point,则将current_waypoint++.如已到达最后一个航点 (9 号)，则输出完成信息，并将其重置为 1 号航点，如此循环。

### int main
初始化ROS2，创建NavNode节点，关闭ROS2（ctrl+c）

### 宏观机器人路线
地图任意点-1-2...-9-1-...
