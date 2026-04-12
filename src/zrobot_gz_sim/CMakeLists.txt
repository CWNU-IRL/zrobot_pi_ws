cmake_minimum_required(VERSION 3.8)
project(zrobot_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rs_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gazebo_motor_bridge_node src/gazebo_motor_bridge_node.cpp)
ament_target_dependencies(gazebo_motor_bridge_node
  rclcpp
  rs_interface
  sensor_msgs
  std_msgs
)

install(TARGETS gazebo_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
