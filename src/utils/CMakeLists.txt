cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(tf_trans src/tf_trans.cpp)
set(ROSBAG_PLAYER "rosbag_player")
set(ROSBAG_LISTENER "rosbag_listener")
ament_auto_add_library(${ROSBAG_PLAYER} SHARED src/rosbag_player.cpp)
ament_auto_add_library(${ROSBAG_LISTENER} SHARED src/rosbag_listener.cpp)
target_link_libraries(tf_trans fmt::fmt yaml-cpp)

# 注册rosplay_bag为一个组件
rclcpp_components_register_node(${ROSBAG_PLAYER}
  PLUGIN "RosbagPlayer"
  EXECUTABLE ${ROSBAG_PLAYER}_node
)
rclcpp_components_register_node(${ROSBAG_LISTENER}
  PLUGIN "RosbagListener"
  EXECUTABLE ${ROSBAG_LISTENER}_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()