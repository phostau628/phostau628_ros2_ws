#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class ImuNode :public rclcpp::Node
{
  public:ImuNode() :Node("imu_behavior")
  { 
    vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    imu_sub = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&ImuNode::IMUCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ImuNode::timer_callback ,this)
    );
  }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist vel_msg;
  
    double diff_angle = 0.0;
    void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
      tf2::Quaternion tf2_quaternion;
      tf2_quaternion.setX(msg->orientation.x);
      tf2_quaternion.setY(msg->orientation.y);
      tf2_quaternion.setZ(msg->orientation.z);
      tf2_quaternion.setW(msg->orientation.w);

      tf2::Matrix3x3 matrix(tf2_quaternion);

      double roll, pitch, yaw;
      matrix.getRPY(roll, pitch, yaw);
      roll = roll * 180 / M_PI;
      pitch = pitch * 180 / M_PI;
      yaw = yaw * 180 / M_PI;
      RCLCPP_INFO(this->get_logger(), "roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);

      double target_yaw = 90;
      
      diff_angle = target_yaw - yaw;
      diff_angle = fmod(diff_angle + 180.0 , 360.0) - 180.0;
      
    
    
    
}

  void timer_callback()
  { 
    vel_msg.angular.z = diff_angle * 0.01;
    vel_msg.linear.x = 1.0;
    vel_pub->publish(vel_msg);
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