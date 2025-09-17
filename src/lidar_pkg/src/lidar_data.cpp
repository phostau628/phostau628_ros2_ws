#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>  

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_data")
    {
       
        lidar_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&LidarNode::LidarCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Lidar node initialized and subscribed to /scan topic");
    }

private:
    // 订阅者作为类成员变量（在回调函数外声明）
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    
    void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int nNum = msg->ranges.size();
        
        if (nNum == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty laser scan data");
            return;
        }

        int nMid = nNum / 2;
        float fMidDist = msg->ranges[nMid];
        
        // 检查距离是否有效
        if (std::isnan(fMidDist) || fMidDist < msg->range_min || fMidDist > msg->range_max) {
            RCLCPP_INFO(this->get_logger(), "ranges[%d] = invalid", nMid);
        } else {
            RCLCPP_INFO(this->get_logger(), "ranges[%d] = %f m", nMid, fMidDist);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}