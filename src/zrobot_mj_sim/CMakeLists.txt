cmake_minimum_required(VERSION 3.8)
project(zrobot_mj_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rs_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(MUJOCO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/mujoco-3.9.0")

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS "${MUJOCO_ROOT}"
  PATH_SUFFIXES include)

find_library(MUJOCO_LIBRARY mujoco
  HINTS "${MUJOCO_ROOT}"
  PATH_SUFFIXES lib lib64)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo not found at ${MUJOCO_ROOT}. Download precompiled package to third_party/mujoco-3.9.0/")
endif()

add_executable(mujoco_motor_bridge_node
  src/mujoco_motor_bridge_node.cpp
)

target_include_directories(mujoco_motor_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

ament_target_dependencies(mujoco_motor_bridge_node
  rclcpp
  rs_interface
  sensor_msgs
  std_msgs
  rosgraph_msgs
  ament_index_cpp
)

get_filename_component(MUJOCO_LIBRARY_DIR "${MUJOCO_LIBRARY}" DIRECTORY)

target_link_libraries(mujoco_motor_bridge_node
  ${MUJOCO_LIBRARY}
)

target_link_options(mujoco_motor_bridge_node PRIVATE
  "-Wl,-rpath,${MUJOCO_LIBRARY_DIR}"
)

install(TARGETS mujoco_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
