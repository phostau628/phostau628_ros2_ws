#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>
#include <cmath>

class TurtleRMNode : public rclcpp::Node
{
public:
    TurtleRMNode() : Node("turtle_rm")
    {
      
        vel_pub = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        
       
        pose_sub = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", rclcpp::SensorDataQoS(), 
            std::bind(&TurtleRMNode::poseCallback, this, std::placeholders::_1)
        );
        
     
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TurtleRMNode::timerCallback, this)
        );

        // 点列表
        target_points = {
            {5.54, 8.54}, {6.54, 7.54}, {5.54, 6.54}, {6.54, 5.54},
            {7.04, 8.54}, {7.54, 5.54}, {7.84, 8.54}, {8.54, 5.54}
        };
        
        current_target_idx = 0;
        RCLCPP_INFO(this->get_logger(), "小PhosTau启动!");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist vel_msg;
    
    turtlesim::msg::Pose current_pose;
    std::vector<std::pair<double, double>> target_points;
    size_t current_target_idx;
    const double min_linear_vel = 0.1; 
    const double position_tolerance = 0.02;  // 误差

    void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose = *msg;
    }

    // 算两点距离
    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)); 
    }

    // 算目标角
    double calculateTargetAngle(double target_x, double target_y)
    {
        double dx = target_x - current_pose.x;
        double dy = target_y - current_pose.y;
        return atan2(dy, dx);
    }

    // 算角度差
    double calculateAngleError(double target_angle)
    {
        double error = target_angle - current_pose.theta;
        
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI; //取捷径
        return error;
    }

    void timerCallback()
    {
        // 检查任务是否全部完成
        if (current_target_idx >= target_points.size())
        {
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0.0;
            vel_pub->publish(vel_msg);
            RCLCPP_INFO(this->get_logger(), "PhosTau滴任务完成了!!!");
            return;
        }

        // 获取下一个目标点坐标
        double target_x = target_points[current_target_idx].first;
        double target_y = target_points[current_target_idx].second;

        // 计算到目标点的距离
        double distance = calculateDistance(
            current_pose.x, current_pose.y,
            target_x, target_y
        );

        // 检查是否到达当前目标点
        if (distance < position_tolerance)
        {
            RCLCPP_INFO(this->get_logger(), "到达%zu号点 : (%.2f, %.2f)",
                        current_target_idx + 1, target_x, target_y);
            current_target_idx++;
            return;
        }

       
        double target_angle = calculateTargetAngle(target_x, target_y);
        double angle_error = calculateAngleError(target_angle);

       
        if (fabs(angle_error) > 0.06)  
        {
            vel_msg.angular.z = angle_error * 3.5;  
            vel_msg.linear.x = 0.0;
        }
        else
        {
            vel_msg.angular.z = 0.0;
            vel_msg.linear.x = std::min(distance * 1.57, 2.4);  // 考虑到实体车需求：平滑刹车
            vel_msg.linear.x = std::max(vel_msg.linear.x, min_linear_vel);
        }
        vel_pub->publish(vel_msg);
    }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleRMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
