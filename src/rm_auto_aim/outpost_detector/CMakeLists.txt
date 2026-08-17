cmake_minimum_required(VERSION 3.10)
project(outpost_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(THIS_LIBRARY ${PROJECT_NAME})
ament_auto_add_library(${THIS_LIBRARY} SHARED
  src/outpost_target_node.cpp
)

target_include_directories(${THIS_LIBRARY} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# EKF 直接复用 armor_tracker 的 ExtendedKalmanFilter
ament_target_dependencies(${THIS_LIBRARY}
  armor_tracker
)

rclcpp_components_register_node(${THIS_LIBRARY}
  PLUGIN outpost_auto_aim::OutpostTargetNode
  EXECUTABLE outpost_target_node
)

install(PROGRAMS scripts/fake_outpost.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
