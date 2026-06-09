cmake_minimum_required(VERSION 3.8)
project(control)

# Default to C++14 if no standard is specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fs_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Declaração dos executáveis
add_executable(check_node ros2/check_node.cpp)
add_executable(check_node_lifecycle ros2/check_node_lifecycle.cpp)

# Dependências do check_node
ament_target_dependencies(check_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  fs_msgs
  lifecycle_msgs
)

# CORREÇÃO AQUI: Faltava vincular as dependências para o seu nó de ciclo de vida!
ament_target_dependencies(check_node_lifecycle
  rclcpp
  rclcpp_lifecycle
  std_msgs
  fs_msgs
  lifecycle_msgs
)

# Adicione também o check_node na instalação para ele poder ser executado
install(TARGETS
  check_node
  check_node_lifecycle
  DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY
        ros2
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