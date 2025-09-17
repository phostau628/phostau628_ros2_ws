#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>  // 用于NaN检查

// 激光雷达行为节点类（避障逻辑）
class LidarBehaviorNode : public rclcpp::Node
{
public:
    // 构造函数：初始化节点、订阅者、发布者、定时器
    LidarBehaviorNode() : Node("lidar_behavior")
    {
        // 1. 创建速度发布者（发布/cmd_vel控制机器人）
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // 2. 创建激光雷达订阅者（接收/scan数据）
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&LidarBehaviorNode::lidar_callback, this, std::placeholders::_1)
        );
        // 3. 创建定时器（100ms触发一次，控制运动频率）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz控制频率，比1Hz更流畅
            std::bind(&LidarBehaviorNode::timer_callback, this)
        );

        // 初始化日志（确认节点启动）
        RCLCPP_INFO(this->get_logger(), "Lidar Behavior Node Started!");
    }

private:
    // -------------------------- 成员变量 --------------------------
    // 激光雷达订阅者（显式声明类型，兼容所有C++17环境）
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    // 速度指令发布者（显式声明类型，避免推导错误）
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    // 定时器（显式声明类型）
    rclcpp::TimerBase::SharedPtr timer_;
    // 速度指令消息（存储要发布的速度）
    geometry_msgs::msg::Twist vel_msg_;
    // 避障计数（控制旋转时间）
    int avoid_count_ = 0;
    // 正前方距离（存储激光雷达检测到的正前方距离）
    float front_dist_ = NAN;
    // 激光雷达有效距离范围（从激光消息中获取）
    float range_min_ = 0.0f;
    float range_max_ = 0.0f;

    // -------------------------- 回调函数 --------------------------
    // 1. 激光雷达回调：处理/scan数据，提取正前方距离
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 保存激光雷达的有效距离范围（避免每次判断都读msg）
        range_min_ = msg->range_min;
        range_max_ = msg->range_max;

        // 检查激光数据是否为空（防止数组越界）
        if (msg->ranges.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty laser data received!");
            front_dist_ = NAN;  // 标记为无效距离
            return;
        }

        // 计算正前方激光束的索引（假设激光是对称扫描，中间点为正前方）
        int mid_idx = msg->ranges.size() / 2;
        // 获取正前方距离
        float raw_dist = msg->ranges[mid_idx];

        // 检查距离是否有效（排除NaN、超出最小/最大范围的情况）
        if (std::isnan(raw_dist) || raw_dist < range_min_ || raw_dist > range_max_)
        {
            front_dist_ = NAN;
            RCLCPP_INFO(this->get_logger(), "Front distance invalid (out of range/NaN)");
        }
        else
        {
            front_dist_ = raw_dist;
            RCLCPP_DEBUG(this->get_logger(), "Front distance: %.2f m", front_dist_);  // DEBUG级日志，不干扰正常输出
        }
    }

    // 2. 定时器回调：根据正前方距离决策运动（10Hz触发）
    void timer_callback()
    {
        // 先判断是否有有效距离数据（没有则停止运动）
        if (std::isnan(front_dist_))
        {
            vel_msg_.linear.x = 0.0f;    // 停止前进
            vel_msg_.angular.z = 0.0f;   // 停止旋转
            vel_pub_->publish(vel_msg_);
            return;
        }

        // 避障逻辑：正前方距离小于1.5米时旋转避障
        if (front_dist_ < 1.5f)
        {
            // 控制旋转时间：avoid_count_=50 → 50*100ms=5秒旋转
            if (avoid_count_ == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected! Rotating...");
                avoid_count_ = 50;  // 重置旋转计数
            }

            // 执行旋转（只转不前进，避免碰撞）
            vel_msg_.linear.x = 0.0f;    // 停止前进
            vel_msg_.angular.z = 0.3f;   // 顺时针旋转（正数为逆时针，负数为顺时针，可根据机器人方向调整）
            avoid_count_--;  // 计数递减
        }
        // 无障碍物：正常前进
        else
        {
            vel_msg_.linear.x = 0.2f;    // 前进速度（0.2m/s，避免太快失控）
            vel_msg_.angular.z = 0.0f;   // 不旋转（直线前进）
            avoid_count_ = 0;  // 重置避障计数（下次有障碍可重新计时）
        }

        // 发布速度指令（控制机器人运动）
        vel_pub_->publish(vel_msg_);
    }
};

// -------------------------- 主函数 --------------------------
int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建节点实例
    auto node = std::make_shared<LidarBehaviorNode>();
    // 自旋节点（阻塞，处理回调）
    rclcpp::spin(node);
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}