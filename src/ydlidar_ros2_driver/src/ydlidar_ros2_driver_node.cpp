/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  // 1. 字符串类型参数：补充默认值（标准类型）
  node->declare_parameter("port", "/dev/ydlidar");
  node->get_parameter("port", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  str_optvalue = "";
  node->declare_parameter("ignore_array", "");
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", "laser_frame");
  node->get_parameter("frame_id", frame_id);

  //////////////////////int 类型参数（关键修复：枚举转int）/////////////////
  int optval = 512000;
  // 波特率（int，标准类型）
  node->declare_parameter("baudrate", 512000);
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  // 激光雷达类型（TYPE_TRIANGLE是枚举，转int传参）
  optval = static_cast<int>(TYPE_TRIANGLE);
  node->declare_parameter("lidar_type", static_cast<int>(TYPE_TRIANGLE));
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  // 设备类型（YDLIDAR_TYPE_SERIAL是枚举，转int传参）
  optval = static_cast<int>(YDLIDAR_TYPE_SERIAL);
  node->declare_parameter("device_type", static_cast<int>(YDLIDAR_TYPE_SERIAL));
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

  // 采样率（int，标准类型）
  optval = 9;
  node->declare_parameter("sample_rate", 9);
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

  // 异常检查次数（int，标准类型）
  optval = 4;
  node->declare_parameter("abnormal_check_count", 4);
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
     

  //////////////////////bool 类型参数（标准类型，无问题）/////////////////
  bool b_optvalue = false;
  // 固定分辨率
  node->declare_parameter("fixed_resolution", false);
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));

  // 180度反转
  b_optvalue = true;
  node->declare_parameter("reversion", true);
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));

  // 逆时针旋转
  b_optvalue = true;
  node->declare_parameter("inverted", true);
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));

  // 自动重连
  b_optvalue = true;
  node->declare_parameter("auto_reconnect", true);
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  // 单通道模式
  b_optvalue = false;
  node->declare_parameter("isSingleChannel", false);
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));

  // 强度信息
  b_optvalue = false;
  node->declare_parameter("intensity", false);
  node->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));

  // 电机DTR控制
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr", false);
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float 类型参数（标准类型，无问题）/////////////////
  float f_optvalue = 180.0f;
  // 最大角度（度）
  node->declare_parameter("angle_max", 180.0);
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));

  // 最小角度（度）
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min", -180.0);
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  // 最大测距（米）
  f_optvalue = 64.f;
  node->declare_parameter("range_max", 12.0);
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));

  // 最小测距（米）
  f_optvalue = 0.1f;
  node->declare_parameter("range_min", 0.1);
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

  // 扫描频率（Hz）
  f_optvalue = 10.f;
  node->declare_parameter("frequency", 10.0);
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  // 无效距离设为无穷
  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", false);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);


  // 初始化激光雷达
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  // 创建激光雷达数据发布者
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::KeepLast(10)));

  // 停止扫描服务（消除未使用参数警告）
  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    (void)request_header;  // 标记未使用参数，消除警告
    (void)req;
    (void)response;
    return laser.turnOff();
  };
  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

  // 启动扫描服务（消除未使用参数警告）
  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    (void)request_header;  // 标记未使用参数，消除警告
    (void)req;
    (void)response;
    return laser.turnOn();
  };
  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

  // 循环发布激光雷达数据
  rclcpp::WallRate loop_rate(20);
  while (ret && rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      // 填充时间戳
      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      // 填充扫描参数
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      // 填充测距数据和强度数据
      int size = static_cast<int>((scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1);
      scan_msg->ranges.resize(size, scan_msg->range_max + 1.0);  // 初始化无效值
      scan_msg->intensities.resize(size, 0.0);
      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = static_cast<int>(std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment));
        if (index >= 0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }
      // 发布数据
      laser_pub->publish(*scan_msg);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    // 处理回调
    if (!rclcpp::ok()) break;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  // 关闭激光雷达并退出
  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}