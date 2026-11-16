cmake_minimum_required(VERSION 3.16)
project(battery_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

# 接口定义
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryStatus.msg"
  "srv/GetStatus.srv"
  "srv/ForceShutdown.srv"
  "srv/ClearFault.srv"
)

# 复用仓库根目录 cpp/ 下的 bms_can 核心库（协议唯一实现来源，
# 与独立 CLI 共用同一份代码）。可通过 -DBMS_CORE_DIR=/path/to/cpp 覆盖。
set(BMS_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../cpp" CACHE PATH
    "bms_can C++ 核心库目录（含 include/bms_can 与 src）")
if(NOT EXISTS "${BMS_CORE_DIR}/include/bms_can/battery_client.h")
  message(FATAL_ERROR "未找到 bms_can 核心库: ${BMS_CORE_DIR}（可用 -DBMS_CORE_DIR 指定）")
endif()

add_library(bms_can_core STATIC
  "${BMS_CORE_DIR}/src/protocol.cpp"
  "${BMS_CORE_DIR}/src/battery_client.cpp"
)
target_include_directories(bms_can_core PUBLIC "${BMS_CORE_DIR}/include")
target_link_libraries(bms_can_core PUBLIC Threads::Threads)

# 节点
add_executable(battery_node src/battery_node.cpp)
target_link_libraries(battery_node bms_can_core)
ament_target_dependencies(battery_node rclcpp std_msgs)
rosidl_target_interfaces(battery_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_compile_options(battery_node PRIVATE -Wall -Wextra)

install(TARGETS battery_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
