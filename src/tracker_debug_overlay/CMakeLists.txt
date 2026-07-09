cmake_minimum_required(VERSION 3.8)
project(tracker_debug_overlay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(tracker_overlay_node
        src/tracker_overlay_node.cpp
)

ament_target_dependencies(tracker_overlay_node
        rclcpp
        sensor_msgs
        geometry_msgs
        auto_aim_interfaces
        cv_bridge
        image_transport
        message_filters
        tf2
        tf2_ros
        tf2_geometry_msgs
)

target_link_libraries(tracker_overlay_node
        ${OpenCV_LIBS}
        Eigen3::Eigen
)

install(TARGETS tracker_overlay_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()