cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## C++17：onnxruntime 的 C++ 头文件用到了结构化绑定，必须 17 及以上
set(CMAKE_CXX_STANDARD 17)
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

###########################
## onnxruntime（神经网络模式用）##
###########################

## 不额外安装：优先用 ARMOR_ONNXRUNTIME_ROOT 指定，
## 否则复用 buff_detector 里随仓库分发的那份，最后看本包的 third_party。
## 找不到也不影响编译，只是神经网络模式不可用，传统模式照常。
set(ARMOR_ONNXRUNTIME_ROOT "" CACHE PATH "onnxruntime 根目录（含 include/ 与 lib/）")
if(ARMOR_ONNXRUNTIME_ROOT AND NOT EXISTS "${ARMOR_ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
  message(WARNING "armor_detector: ${ARMOR_ONNXRUNTIME_ROOT} 里没有 include/onnxruntime_cxx_api.h，改为自动查找")
  set(ARMOR_ONNXRUNTIME_ROOT "")
endif()
if(NOT ARMOR_ONNXRUNTIME_ROOT)
  foreach(candidate
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime"
      "${CMAKE_CURRENT_SOURCE_DIR}/../buff_detector/third_party/onnxruntime")
    if(EXISTS "${candidate}/include/onnxruntime_cxx_api.h")
      set(ARMOR_ONNXRUNTIME_ROOT "${candidate}")
      break()
    endif()
  endforeach()
endif()

if(ARMOR_ONNXRUNTIME_ROOT)
  message(STATUS "armor_detector: 使用 onnxruntime ${ARMOR_ONNXRUNTIME_ROOT}")
  add_library(onnxruntime SHARED IMPORTED)
  set_target_properties(onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ARMOR_ONNXRUNTIME_ROOT}/lib/libonnxruntime.so"
    INTERFACE_INCLUDE_DIRECTORIES "${ARMOR_ONNXRUNTIME_ROOT}/include"
  )
else()
  message(WARNING "armor_detector: 没找到 onnxruntime，神经网络模式不可用（传统模式不受影响）")
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if(ARMOR_ONNXRUNTIME_ROOT)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ARMOR_DETECTOR_WITH_ONNXRUNTIME)
  target_link_libraries(${PROJECT_NAME} onnxruntime)
  ## 运行时找 libonnxruntime.so：build 时指向源码目录，install 后用 $ORIGIN 找同目录
  set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${ARMOR_ONNXRUNTIME_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN"
  )
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Install ##
#############

if(ARMOR_ONNXRUNTIME_ROOT)
  ## 库的 SONAME 是 libonnxruntime.so.1，安装必须保留该文件名，再建符号链接方便链接查找
  file(GLOB ARMOR_ONNXRUNTIME_LIBS "${ARMOR_ONNXRUNTIME_ROOT}/lib/libonnxruntime.so.1.*")
  list(LENGTH ARMOR_ONNXRUNTIME_LIBS ARMOR_ONNXRUNTIME_LIBS_COUNT)
  if(ARMOR_ONNXRUNTIME_LIBS_COUNT GREATER 0)
    list(GET ARMOR_ONNXRUNTIME_LIBS 0 ARMOR_ONNXRUNTIME_LIB)
    install(FILES "${ARMOR_ONNXRUNTIME_LIB}"
      DESTINATION lib
      RENAME libonnxruntime.so.1
    )
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink libonnxruntime.so.1 \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libonnxruntime.so\")")
  endif()
endif()

## 离线测试程序：不依赖 ROS 运行环境，直接喂图片/视频看神经网络效果
ament_auto_add_executable(test_neural_detector tools/test_neural_detector.cpp)
target_include_directories(test_neural_detector PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_neural_detector ${PROJECT_NAME} ${OpenCV_LIBS})

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
