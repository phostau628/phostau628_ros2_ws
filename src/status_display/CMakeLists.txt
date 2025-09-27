cmake_minimum_required(VERSION 3.8)
project(status_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找所有依赖（ROS 2 + Qt）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)          # ROS 2 C++ 核心库
find_package(status_interfaces REQUIRED)# 自定义消息包
find_package(Qt5 REQUIRED COMPONENTS Widgets)  # Qt Widgets 库

# 2. 添加可执行文件
add_executable(status_display src/status_display.cpp)

# 3. 关键：关联 ROS 2 依赖（传递头文件路径和库）
ament_target_dependencies(status_display
  "rclcpp"
  status_interfaces
  
)

# 4. 关联 Qt 依赖（原配置正确，保留）
target_link_libraries(status_display Qt5::Widgets)

# 5. 安装可执行文件
install(TARGETS status_display
    DESTINATION lib/${PROJECT_NAME})

# 6. 测试配置（保留）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()