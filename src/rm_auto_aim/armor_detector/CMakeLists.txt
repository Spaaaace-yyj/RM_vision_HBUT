cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

## ===== 内嵌相机模式：链接 MindVision SDK =====
## use_embedded_camera:=true 时，检测节点自己调 SDK 取图，跳过 /image_raw 话题。
## 找不到 SDK 只告警，不影响原来的订阅模式。
set(MVSDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../driver/mindvision_camera/mvsdk")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(MVSDK_LIBDIR "${MVSDK_ROOT}/lib/arm64")
else()
  set(MVSDK_LIBDIR "${MVSDK_ROOT}/lib/amd64")
endif()
if(EXISTS "${MVSDK_LIBDIR}/libMVSDK.so")
  message(STATUS "armor_detector: 内嵌相机模式可用（${MVSDK_LIBDIR}）")
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_WITH_CAMERA)
  target_include_directories(${PROJECT_NAME} PRIVATE "${MVSDK_ROOT}/include")
  target_link_libraries(${PROJECT_NAME} "${MVSDK_LIBDIR}/libMVSDK.so")
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${MVSDK_LIBDIR}"
    INSTALL_RPATH "$ORIGIN")
  install(FILES "${MVSDK_LIBDIR}/libMVSDK.so" DESTINATION lib)
else()
  message(WARNING "armor_detector: 未找到 MindVision SDK（${MVSDK_LIBDIR}），内嵌相机模式不可用")
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
