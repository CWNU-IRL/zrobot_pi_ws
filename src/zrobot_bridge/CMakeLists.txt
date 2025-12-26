cmake_minimum_required(VERSION 3.8)
project(zrobot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BatchMotorPosition.srv"
)

add_executable(motor_service_node
  src/motor_service_node.cpp
  src/motor_cfg.cpp
)
target_include_directories(motor_service_node PRIVATE include)
ament_target_dependencies(motor_service_node rclcpp)
target_link_libraries(motor_service_node Threads::Threads)
rosidl_target_interfaces(motor_service_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS motor_service_node
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
