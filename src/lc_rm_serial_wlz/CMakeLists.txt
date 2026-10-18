cmake_minimum_required(VERSION 3.8)
project(lc_rm_serial_wlz)

# ================= 编译器设置 =================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= 查找依赖 =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(serial_driver REQUIRED)

# ================= 构建可执行文件 =================
add_executable(lc_serial_test_node
  src/lc_serial_test.cpp
  src/crc_check.cpp
  src/serial_process.cpp
)

# ---------------- 头文件搜索路径 ----------------
# 你的代码里用的是 #include "lc_rm_serial_wlz/include/test/xxx.hpp"
# 所以这里必须加上父目录（..），让编译器能找到 lc_rm_serial_wlz 这一层
target_include_directories(lc_serial_test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---------------- 链接依赖库 ----------------
ament_target_dependencies(lc_serial_test_node
  rclcpp
  geometry_msgs
  sensor_msgs
  auto_aim_interfaces
  serial_driver
)

# ================= 安装规则 =================
install(TARGETS lc_serial_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# (可选) 如果你的包里还有其他头文件需要导出给别人用，加上这段：
# install(DIRECTORY include/
#   DESTINATION include/
# )

# ================= 测试规则 =================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ================= 收尾 =================
ament_package()