#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <vector>
#include <cmath>
#include <string>

class PatrolControllerNode : public rclcpp::Node {
public:
    // 构造函数：初始化所有组件和参数
    PatrolControllerNode() : Node("patrol_controller") {
        // 声明并获取参数
        this->declare_parameter("obstacle_threshold", 0.5);
        this->declare_parameter("movement_speed", 0.2);
        this->declare_parameter("rotation_speed", 0.5);
        
        this->get_parameter("obstacle_threshold", obstacle_threshold_);
        this->get_parameter("movement_speed", movement_speed_);
        this->get_parameter("rotation_speed", rotation_speed_);

        // 初始化发布者、订阅者和服务
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PatrolControllerNode::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PatrolControllerNode::odom_callback, this, std::placeholders::_1));
        status_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/get_status", std::bind(&PatrolControllerNode::status_callback, this, 
            std::placeholders::_1, std::placeholders::_2));

        // 添加初始巡逻点
        add_waypoint(1.0, 0.0);
        add_waypoint(1.0, 1.0);
        add_waypoint(0.0, 1.0);

        // 创建控制循环定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz控制频率
            std::bind(&PatrolControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "巡逻控制器初始化完成");
    }

    // 公开方法：添加新的巡逻点
    void add_waypoint(float x, float y) {
        waypoints_.emplace_back(x, y);
        RCLCPP_INFO(this->get_logger(), "添加新目标点: (%.2f, %.2f)", x, y);
    }

    // 公开方法：清除所有巡逻点
    void clear_waypoints() {
        waypoints_.clear();
        current_waypoint_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "已清除所有目标点");
    }

private:
    // 私有成员变量：状态集中管理
    std::vector<std::pair<float, float>> waypoints_;  // 巡逻点列表
    int current_waypoint_idx_ = 0;                    // 当前目标点索引
    float current_x_ = 0.0, current_y_ = 0.0;         // 当前位置
    bool obstacle_detected_ = false;                  // 障碍物检测状态
    float obstacle_threshold_;                        // 障碍物阈值参数
    float movement_speed_;                            // 移动速度参数
    float rotation_speed_;                            // 旋转速度参数

    // 私有成员：ROS通信组件
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;       // 控制循环定时器

    // 激光雷达回调：检测障碍物
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacle_detected_ = false;
        // 只检测前方±30度范围内的障碍物
        int start_idx = msg->ranges.size() * 0.35;
        int end_idx = msg->ranges.size() * 0.65;
        
        for (int i = start_idx; i < end_idx; ++i) {
            float dist = msg->ranges[i];
            if (!std::isnan(dist) && dist < obstacle_threshold_ && dist >= msg->range_min) {
                obstacle_detected_ = true;
                break;
            }
        }
    }

    // 里程计回调：更新当前位置
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
    }

    // 状态查询服务回调
    void status_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = true;
        if (waypoints_.empty()) {
            res->message = "无目标点，等待指令";
        } else {
            auto [target_x, target_y] = waypoints_[current_waypoint_idx_];
            res->message = "当前状态: 巡逻中\n"
                          "当前目标点: (" + std::to_string(target_x) + ", " + std::to_string(target_y) + ")\n"
                          "当前位置: (" + std::to_string(current_x_) + ", " + std::to_string(current_y_) + ")\n"
                          "障碍物状态: " + (obstacle_detected_ ? "检测到障碍物" : "正常");
        }
    }

    // 控制循环：处理移动逻辑
    void control_loop() {
        if (waypoints_.empty()) return;

        auto twist = geometry_msgs::msg::Twist();
        auto [target_x, target_y] = waypoints_[current_waypoint_idx_];

        // 检查是否到达当前目标点
        if (is_near_target(target_x, target_y)) {
            current_waypoint_idx_ = (current_waypoint_idx_ + 1) % waypoints_.size();
            RCLCPP_INFO(this->get_logger(), "到达目标点，切换到下一个目标点");
            return;
        }

        // 避障逻辑
        if (obstacle_detected_) {
            twist.angular.z = rotation_speed_;  // 旋转避障
            twist.linear.x = 0.0;
            RCLCPP_WARN(this->get_logger(), "检测到障碍物，正在绕行");
        } else {
            // 正常移动到目标点
            float angle = calculate_target_angle(target_x, target_y);
            twist.angular.z = std::clamp(angle, -rotation_speed_, rotation_speed_);
            twist.linear.x = movement_speed_ * (1 - std::abs(angle) / M_PI);
        }

        vel_pub_->publish(twist);
    }

    // 辅助函数：判断是否接近目标点
    bool is_near_target(float x, float y, float tolerance = 0.1) {
        float dx = x - current_x_;
        float dy = y - current_y_;
        return std::hypot(dx, dy) < tolerance;
    }

    // 辅助函数：计算到目标点的角度
    float calculate_target_angle(float x, float y) {
        float dx = x - current_x_;
        float dy = y - current_y_;
        return std::atan2(dy, dx);  // 返回目标点相对于当前位置的角度
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolControllerNode>();
    
    // 可以动态添加更多目标点
    node->add_waypoint(0.0, 0.0);  // 回到起点
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
