#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NavNode:public rclcpp::Node
{
    public:
    NavNode()
        :Node("waypoint_navigation"),
        first_pub(true),
        current_waypoint(1),
        new_waypoint_needed(false),
        max_point(9)
        
    {
        navigation_pub = this->create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint",rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
        result_sub = this->create_subscription<std_msgs::msg::String>("/waterplus/navi_result",rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            std::bind(&NavNode::NavCallback, this, std::placeholders::_1));
        timer_=create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavNode::NavPub,this)
        );
    }
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub;
        rclcpp::TimerBase::SharedPtr timer_;
        std_msgs::msg::String waypoint_msg;
        bool first_pub;
        int current_waypoint;
        bool new_waypoint_needed;
        int max_point;

        void NavPub()
        {   
            if (first_pub)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                first_pub = false;
                new_waypoint_needed = true;
            }
            if (new_waypoint_needed && current_waypoint<=max_point)
            {
                waypoint_msg.data = std::to_string(current_waypoint);
                navigation_pub->publish(waypoint_msg);
                RCLCPP_INFO(this->get_logger(), "Published waypoint: %d", current_waypoint);
                new_waypoint_needed = false;
            }
        }
         void NavCallback(const std_msgs::msg::String::SharedPtr msg)
        {   
            RCLCPP_INFO(this->get_logger(), "Received nav result: %s", msg->data.c_str()); // 新增日志
            if(msg->data =="navi done")
            {
                RCLCPP_INFO(this->get_logger(),"ARRIVED AT WAYPOINT %d!",current_waypoint);
                if (current_waypoint < max_point)
              {
                current_waypoint++;  // 切换到下一个航点
                new_waypoint_needed = true;  // 标记需要发布新航点
                RCLCPP_INFO(this->get_logger(), "Proceeding to waypoint %d", current_waypoint);
              }
                else if(current_waypoint == max_point)
              {
                RCLCPP_INFO(this->get_logger(), "All waypoints reached! Navigation completed.");
                current_waypoint = 1;
                new_waypoint_needed = true;
    
                
                        
                        
              }
            }
        }
};

int main(int argc,char ** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<NavNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>

// class WaypointNavigationNode : public rclcpp::Node
// {
// public:
//   WaypointNavigationNode()
//     : Node("waypoint_navigation_node"),
//       current_waypoint_(1),
//       max_waypoints_(3),
//       is_navigating_(false)
//   {
//     // 创建发布者和订阅者
//     navigation_pub_ = create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);
//     result_sub_ = create_subscription<std_msgs::msg::String>(
//       "/waterplus/navi_result", 10, 
//       std::bind(&WaypointNavigationNode::ResultCallback, this, std::placeholders::_1));
    
//     // 使用定时器而不是立即发布
//     timer_ = create_wall_timer(
//       std::chrono::milliseconds(1000),
//       std::bind(&WaypointNavigationNode::TimerCallback, this)
//     );
    
//     RCLCPP_INFO(get_logger(), "Waypoint navigation node started");
//   }

// private:
//   void TimerCallback()
//   {
//     if (!is_navigating_ && current_waypoint_ <= max_waypoints_)
//     {
//       // 发布下一个航点
//       std_msgs::msg::String waypoint_msg;
//       waypoint_msg.data = std::to_string(current_waypoint_);
//       navigation_pub_->publish(waypoint_msg);
      
//       RCLCPP_INFO(get_logger(), "Published waypoint: %s", waypoint_msg.data.c_str());
//       is_navigating_ = true;
//     }
//     else if (current_waypoint_ > max_waypoints_)
//     {
//       RCLCPP_INFO(get_logger(), "All waypoints completed");
//       timer_->cancel(); // 停止定时器
//     }
//   }

//   void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     RCLCPP_INFO(get_logger(), "Received result: %s", msg->data.c_str());
    
//     if (msg->data == "navi done")
//     {
//       RCLCPP_INFO(get_logger(), "Arrived at waypoint %d!", current_waypoint_);
//       current_waypoint_++;
//       is_navigating_ = false;
//     }
//     else if (msg->data.find("failed") != std::string::npos || 
//              msg->data.find("error") != std::string::npos)
//     {
//       RCLCPP_WARN(get_logger(), "Navigation to waypoint %d failed: %s", 
//                  current_waypoint_, msg->data.c_str());
//       // 可以选择重试或继续下一个航点
//       is_navigating_ = false;
//     }
//   }

//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
  
//   int current_waypoint_;
//   int max_waypoints_;
//   bool is_navigating_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<WaypointNavigationNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>

// class WaypointNavigationNode : public rclcpp::Node
// {
// public:
//   WaypointNavigationNode()
//     : Node("waypoint_navigation_node")
//   {
//     navigation_pub_ = create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);

//     result_sub_ = create_subscription<std_msgs::msg::String>(
//       "/waterplus/navi_result", 10, std::bind(&WaypointNavigationNode::ResultCallback, this, std::placeholders::_1));

//     rclcpp::sleep_for(std::chrono::milliseconds(1000));

//     std_msgs::msg::String waypoint_msg;
//     waypoint_msg.data = "1";
//     navigation_pub_->publish(waypoint_msg);
//   }

// private:
//   void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     if (msg->data == "navi done")
//     {
//       RCLCPP_INFO(get_logger(), "Arrived !");
//     }
//   }

//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<WaypointNavigationNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }