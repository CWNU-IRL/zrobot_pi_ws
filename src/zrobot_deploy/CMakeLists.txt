cmake_minimum_required(VERSION 3.8)
project(zrobot_deploy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rs_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)

set(ONNXRUNTIME_DIR ${CMAKE_CURRENT_LIST_DIR}/../../third_party/onnxruntime-linux-x64-1.16.3)

include_directories(
        include
)

add_executable(main src/main.cpp
        src/FSM.cpp
        src/FixStand.cpp
        src/Locomotion.cpp
        include/zrobot_deploy/FSM.h
        include/zrobot_deploy/FixStand.h
        include/zrobot_deploy/Locomotion.h
)

ament_target_dependencies(main
        rclcpp
        rs_interface
        std_msgs
        tf2
)

target_link_libraries(main
        ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
)

target_include_directories(main
        PUBLIC
        ${EIGEN3_INCLUDE_DIR}
        /usr/include/eigen3
        /usr/include/onnxruntime
        ${ONNXRUNTIME_DIR}/include
)

install(TARGETS
        main
        DESTINATION lib/${PROJECT_NAME}
)

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
