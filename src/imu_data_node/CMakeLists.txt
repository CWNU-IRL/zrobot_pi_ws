cmake_minimum_required(VERSION 3.8)
project(imu_data_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(imu_msg REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(LIBSERIAL_LIB serial)

# include directories
include_directories(include)

# add executable
add_executable(imu_data_node src/imu_driver_node.cpp)
ament_target_dependencies(imu_data_node
  rclcpp
  sensor_msgs
  imu_msg
)

target_link_libraries(imu_data_node ${LIBSERIAL_LIB})

# install
install(TARGETS imu_data_node
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
