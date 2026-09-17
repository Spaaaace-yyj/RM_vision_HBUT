cmake_minimum_required(VERSION 3.8)
project(image_latency_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

add_executable(image_latency_test_node
  src/image_latency_test.cpp
)

ament_target_dependencies(
  image_latency_test_node
  rclcpp
  sensor_msgs
  image_transport
)

install(
  TARGETS image_latency_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
