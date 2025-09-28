#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class NavNode : public rclcpp::Node
{
public:
    NavNode()
        : Node("waypoint_navigation"),
          first_pub(true),
          current_waypoint(1),
          new_waypoint_needed(false),
          max_point(7)
    {
        // Nav2接口
        waypoint_client = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");

        // 定时器
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&NavNode::NavPub, this));

        RCLCPP_INFO(this->get_logger(), "Nav2 waypoint client initialized");
    }

private:
    rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoint_client;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_pub;
    int current_waypoint;
    bool new_waypoint_needed;
    int max_point;

    // 生成路径点列表（改：替换原自定义话题为 Nav2 标准位姿）
    std::vector<geometry_msgs::msg::PoseStamped> get_waypoints()
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;

        
        std::vector<std::tuple<double, double, double>> points = {
            {-0.5299511551856995, -1.6337765455245972, 1.0},   // 1号点
            {3.205333709716797, -1.4899272918701172, 1.0},     
            {3.5811502933502197, 0.537585973739624, 1.0},      
            {0.5087340474128723, 0.5292295813560486, 1.0},     
            {0.6480451822280884, 2.8712124824523926, 1.0},     
            {3.6834158897399902, 3.0554561614990234, 1.0},     
            {-5.669492244720459, -0.5244210362434387, 1.0},    // 7号点
   
};

        for (const auto& p : points) {
            geometry_msgs::msg::PoseStamped wp;
            wp.header.frame_id = "map"; 
            wp.header.stamp = this->now();
            wp.pose.position.x = std::get<0>(p);
            wp.pose.position.y = std::get<1>(p);
            wp.pose.orientation.w = std::get<2>(p);  
            waypoints.push_back(wp);
        }

        return waypoints;
    }

    // 发送航点
    void NavPub()
    {
        if (first_pub)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            first_pub = false;
            new_waypoint_needed = true;
            RCLCPP_INFO(this->get_logger(), "First publish");
        }

        // 等 Nav2 插件启动
        if (!waypoint_client->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2...");
            return;
        }

        if (new_waypoint_needed && current_waypoint <= max_point)
        {
            // 获取完整路径点列表
            auto all_waypoints = get_waypoints();
            std::vector<geometry_msgs::msg::PoseStamped> target_waypoints;

            // 从当前点开始截取
            for (int i = current_waypoint - 1; i < max_point; ++i) {
                target_waypoints.push_back(all_waypoints[i]);
            }

            // 发送目标航点
            auto goal_msg = FollowWaypoints::Goal();
            goal_msg.poses = target_waypoints;

            auto send_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
            send_options.result_callback = 
                std::bind(&NavNode::result_callback, this, std::placeholders::_1);

            waypoint_client->async_send_goal(goal_msg, send_options);
            RCLCPP_INFO(this->get_logger(), "Published waypoint: %d (total: %zu)", 
                      current_waypoint, target_waypoints.size());
            new_waypoint_needed = false;
        }
    }

    // 导航结果回调（NavCallback）
    void result_callback(const GoalHandleFollowWaypoints::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Received nav result: navi done");
                if (current_waypoint < max_point)
                {
                    current_waypoint++;
                    new_waypoint_needed = true;
                    RCLCPP_INFO(this->get_logger(), "Proceeding to waypoint %d", current_waypoint);
                }
                else if (current_waypoint == max_point)
                {
                    RCLCPP_INFO(this->get_logger(), "PHOSTAU ACCOMPLISHD! RESTARTING...");
                    current_waypoint = 1;
                    new_waypoint_needed = true;
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted at waypoint %d", current_waypoint);
                new_waypoint_needed = true;  // 失败后重试
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}