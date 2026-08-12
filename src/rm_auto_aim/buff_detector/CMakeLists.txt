cmake_minimum_required(VERSION 3.10)
project(buff_detector)

## 浙大 buff_algo 库要求 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

## onnxruntime CPU 版，随仓库分发，无系统安装
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES
  IMPORTED_LOCATION
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/lib/libonnxruntime.so
  INTERFACE_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/include
)

###########
## Build ##
###########

## 浙大库源码全部编译，排除依赖 TensorRT 的 buff_detector.cpp
file(GLOB BUFF_ALGO_SRC CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/buff_algo/src/*/*.cpp"
)
list(FILTER BUFF_ALGO_SRC EXCLUDE REGEX "detector/buff_detector\\.cpp$")

set(THIS_LIBRARY ${PROJECT_NAME})
ament_auto_add_library(${THIS_LIBRARY} SHARED
  src/onnx_buff_detector.cpp
  src/buff_detector_node.cpp
  ${BUFF_ALGO_SRC}
)

target_include_directories(${THIS_LIBRARY} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/buff_algo/include
)
target_link_libraries(${THIS_LIBRARY}
  onnxruntime
  ${OpenCV_LIBS}
)

## 运行时找 libonnxruntime.so：build 时指向源码目录，install 后用 $ORIGIN 找同目录
set_target_properties(${THIS_LIBRARY} PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/lib"
  INSTALL_RPATH "$ORIGIN"
)

## 组件注册，可组合进一个进程
rclcpp_components_register_node(${THIS_LIBRARY}
  PLUGIN buff_auto_aim::BuffDetectorNode
  EXECUTABLE buff_detector_node
)

## 离线测试程序，不参与 install
ament_auto_add_executable(test_buff_detector src/test_buff_detector.cpp)
target_include_directories(test_buff_detector PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/buff_algo/include
)
target_link_libraries(test_buff_detector ${THIS_LIBRARY} ${OpenCV_LIBS})

ament_auto_add_executable(test_buff_pipeline src/test_buff_pipeline.cpp)
target_include_directories(test_buff_pipeline PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/buff_algo/include
)
target_link_libraries(test_buff_pipeline ${THIS_LIBRARY} ${OpenCV_LIBS})

#############
## Install ##
#############

## 库的 SONAME 是 libonnxruntime.so.1，安装必须保留该文件名，
## 再建 libonnxruntime.so 符号链接方便链接查找
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime/lib/libonnxruntime.so.1.23.2
  DESTINATION lib
  RENAME libonnxruntime.so.1
)
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink libonnxruntime.so.1 \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libonnxruntime.so\")")
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
