cmake_minimum_required(VERSION 3.5)
project(envir_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# 安装模型到 install 目录（供 Gazebo 识别）
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# 安装世界文件到 install 目录
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 可选：添加测试（若不需要可删除）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()