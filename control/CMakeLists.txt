cmake_minimum_required(VERSION 3.8)
project(control)

# Default to C++14 if no standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(check_node ros2/check_node.cpp)
ament_target_dependencies(check_node
  rclcpp
  std_msgs
  fs_msgs
  lifecycle_msgs
)

install(TARGETS check_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
        PROGRAMS 
        ros2/control_node.py
        DESTINATION lib/${PROJECT_NAME}
)
install(
        PROGRAMS 
        ros2/lifecycle_control_node.py
        DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY
        include
        DESTINATION lib/${PROJECT_NAME} 
)

install(
        DIRECTORY
        lateral_control
        DESTINATION lib/${PROJECT_NAME} 
)

install(
        DIRECTORY
        longitudinal_control
        DESTINATION lib/${PROJECT_NAME} 
)
install(
        PROGRAMS 
        ros2/lifecycle_control_v1.py
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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