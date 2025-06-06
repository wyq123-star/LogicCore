cmake_minimum_required(VERSION 3.8)
project(my_algorithm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

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

install(
  PROGRAMS
  src/random_generate.py # 替换为你的Python节点文件路径
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  src/logic.py # 替换为你的Python节点文件路径
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
  src/nav_behave.py # 替换为你的Python节点文件路径
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
