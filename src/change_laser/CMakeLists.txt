cmake_minimum_required(VERSION 3.8)
project(change_laser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED) # 用于Costmap消息
find_package(geometry_msgs REQUIRED) # 用于PoseArray消息

add_executable(laser_change_node src/laser_change.cpp)
ament_target_dependencies(laser_change_node
  rclcpp tf2_ros sensor_msgs)

# 创建可执行文件
add_executable(obstacle_extractor_node
  src/obstacle_extractor.cpp 
)
ament_target_dependencies(obstacle_extractor_node
  rclcpp
  nav2_msgs
  geometry_msgs
)

# 安装规则
install(TARGETS
  obstacle_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS laser_change_node
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
