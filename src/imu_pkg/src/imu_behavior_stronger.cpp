#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>  // 用于std::clamp

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_behavior")
    {
        // 1. 参数化配置（保留优化：支持动态修改参数）
        this->declare_parameter<double>("target_yaw_deg", 90.0);       // 目标偏航角(度)
        this->declare_parameter<double>("angular_gain", 0.01);          // 角度控制增益（和原代码一致）
        this->declare_parameter<double>("linear_speed", 0.1);           // 线速度（和原代码一致）
        this->declare_parameter<double>("max_angular_speed", 0.5);      // 最大角速度限制（新增安全功能）
        this->declare_parameter<double>("angle_tolerance", 2.0);        // 角度容忍范围（新增：到达目标后停止旋转）
        this->declare_parameter<int>("control_rate_ms", 100);           // 控制频率（和原代码一致：10Hz）

        // 获取参数值
        target_yaw_deg_ = this->get_parameter("target_yaw_deg").as_double();
        angular_gain_ = this->get_parameter("angular_gain").as_double();
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        target_yaw_rad_ = target_yaw_deg_ * M_PI / 180.0;  // 转为弧度计算

        // 2. 创建发布者/订阅者（保留QoS优化，但和原代码功能一致）
        // /cmd_vel用可靠传输，确保控制指令不丢失
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable()
        );

        // /imu/data用实时优先传输，和原代码订阅逻辑一致
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",  // 订阅话题和原代码完全一致
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&ImuNode::IMUCallback, this, std::placeholders::_1)
        );

        // 3. 控制定时器（和原代码一致：100ms触发一次）
        auto control_period = std::chrono::milliseconds(
            this->get_parameter("control_rate_ms").as_int()
        );
        control_timer_ = this->create_wall_timer(
            control_period,
            std::bind(&ImuNode::control_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "IMU控制节点初始化完成. 目标偏航角: %.1f度, 线速度: %.1f", 
                    target_yaw_deg_, linear_speed_);
    }

private:
    // 发布者/订阅者/定时器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 控制消息
    geometry_msgs::msg::Twist vel_msg_;

    // 状态变量（新增：跟踪当前偏航角和数据接收状态）
    double current_yaw_rad_ = 0.0;  // 当前偏航角（弧度）
    bool is_imu_received_ = false;  // 是否收到IMU数据（避免初始无数据时乱发指令）

    // 配置参数（参数化存储，支持动态修改）
    double target_yaw_deg_;         // 目标偏航角（度）
    double target_yaw_rad_;         // 目标偏航角（弧度）
    double angular_gain_;           // 角度控制增益
    double linear_speed_;           // 线速度
    double max_angular_speed_;      // 最大角速度限制（防止旋转过快）
    double angle_tolerance_;        // 角度容忍范围（到达目标后停止旋转）

    // 4. IMU回调函数：完全回归原代码逻辑（不做有效性检查，直接解析）
    void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 移除所有有效性检查，直接解析四元数→欧拉角（和原代码一致）
        tf2::Quaternion quat;
        quat.setX(msg->orientation.x);
        quat.setY(msg->orientation.y);
        quat.setZ(msg->orientation.z);
        quat.setW(msg->orientation.w);

        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  // 获取弧度值

        // 保存当前偏航角，标记已收到数据
        current_yaw_rad_ = yaw;
        is_imu_received_ = true;

        // 保留原代码的日志输出（度格式，方便调试）
        double roll_deg = roll * 180 / M_PI;
        double pitch_deg = pitch * 180 / M_PI;
        double yaw_deg = yaw * 180 / M_PI;
        RCLCPP_INFO(this->get_logger(), "roll= %.0f, pitch= %.0f, yaw= %.0f", 
                    roll_deg, pitch_deg, yaw_deg);
    }

    // 5. 控制回调函数：保留优化（角度归一化、角速度限制），逻辑和原代码一致
    void control_callback()
    {
        // 初始无IMU数据时，停止机器人（新增安全逻辑，避免乱发指令）
        if (!is_imu_received_)
        {
            vel_msg_.linear.x = 0.0;
            vel_msg_.angular.z = 0.0;
            vel_pub_->publish(vel_msg_);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "等待IMU数据...");
            return;
        }

        // 计算角度误差（优化：归一化到[-180, 180]度，避免绕远路）
        double current_yaw_deg = current_yaw_rad_ * 180 / M_PI;
        double angle_error_deg = target_yaw_deg_ - current_yaw_deg;
        // 归一化误差（例如：350度→10度，避免机器人逆时针转340度）
        angle_error_deg = fmod(angle_error_deg + 180.0, 360.0) - 180.0;

        // 角度误差在容忍范围内→停止旋转（新增：到达目标后不转）
        if (std::abs(angle_error_deg) < angle_tolerance_)
        {
            vel_msg_.angular.z = 0.0;
            RCLCPP_INFO_ONCE(this->get_logger(), "已到达目标角度！当前误差: %.1f度", angle_error_deg);
        }
        else
        {
            // 计算角速度（和原代码逻辑一致：误差×增益）
            vel_msg_.angular.z = angle_error_deg * angular_gain_;
            // 限制最大角速度（新增：防止旋转过快导致失控）
            vel_msg_.angular.z = std::clamp(vel_msg_.angular.z, 
                                           -max_angular_speed_, 
                                            max_angular_speed_);
        }

        // 线速度（和原代码一致：固定0.1）
        vel_msg_.linear.x = linear_speed_;

        // 发布控制指令
        vel_pub_->publish(vel_msg_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}