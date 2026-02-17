cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)

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

add_executable(aruco_detector src/aruco_detector.cpp src/aruco.cpp)
ament_target_dependencies(aruco_detector
        rclcpp
        OpenCV
        sensor_msgs
        cv_bridge
        tf2
        tf2_ros
        geometry_msgs
        tf2_geometry_msgs
        visualization_msgs
        auto_aim_interfaces
)

install(TARGETS
        aruco_detector
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
