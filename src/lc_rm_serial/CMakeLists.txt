cmake_minimum_required(VERSION 3.8)
project(lc_serial_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(geometry_msgs REQUIRED)


# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

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

add_executable(lc_serial_test src/lc_serial_test.cpp src/crc_check.cpp src/serial_process.cpp)
ament_target_dependencies(lc_serial_test rclcpp serial_driver geometry_msgs std_msgs)

install(TARGETS
  lc_serial_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
