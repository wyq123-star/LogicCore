cmake_minimum_required(VERSION 3.8)
project(my_perception)
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

# 设置 LLD 为链接器
add_link_options("-fuse-ld=lld")

# 设置构建系统生成器（Ninja）
# set(CMAKE_GENERATOR "Ninja" CACHE STRING "Build system generator" FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()
set (ImuTransform "ImuTransform")
ament_auto_add_library(${ImuTransform} SHARED src/imu_transform.cpp)
target_link_libraries(${ImuTransform} Eigen3::Eigen)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
rclcpp_components_register_node(${ImuTransform}
  PLUGIN "ImuTransform"
  EXECUTABLE ${ImuTransform}_node
)
install(
  DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/slam_riqiang.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
