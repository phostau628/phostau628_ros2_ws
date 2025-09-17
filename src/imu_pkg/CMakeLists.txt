cmake_minimum_required(VERSION 3.8)
project(imu_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)  # 新增：补全 TF2 相关依赖
find_package(geometry_msgs REQUIRED)  # 新增：优化代码用了 geometry_msgs::msg::Twist

add_executable(imu_behavior src/imu_behavior.cpp)
target_include_directories(imu_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_executable(imu_behavior_stronger src/imu_behavior_stronger.cpp)
target_include_directories(imu_behavior_stronger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_behavior PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  imu_behavior
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "tf2"
)
target_compile_features(imu_behavior_stronger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  imu_behavior_stronger
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"  # 新增依赖
  "geometry_msgs"  # 新增依赖
)

install(TARGETS imu_behavior
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS imu_behavior_stronger
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
