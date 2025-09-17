#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class VelNode:public rclcpp::Node
{
  public:
  VelNode()
      :Node("vel_cpp_node")
  {
    vel_pub=create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
    timer_=create_wall_timer(
      std::chrono::milliseconds(300),
      std::bind(&VelNode::VelMsg,this)
    );



  }

  private:
  
    void VelMsg()
      {
        vel_msg.linear.x=0.1;
        vel_msg.linear.y=0.1;
        vel_msg.linear.z=0.0;
        vel_msg.angular.x=0.0;
        vel_msg.angular.y=0.0;
        vel_msg.angular.z=0.0;
        vel_pub->publish(vel_msg);
      }
    
    geometry_msgs::msg::Twist vel_msg;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
    rclcpp::TimerBase::SharedPtr timer_; 

  

};
  


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  auto node=std::make_shared<VelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
