cmake_minimum_required(VERSION 3.10)
project(video_pub)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find dependencies by ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
## cv_bridge 只传递 core/imgproc/imgcodecs，cv::VideoCapture 需要 videoio，
## 必须显式链接 OpenCV
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

##
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN video_pub::VideoPub
  EXECUTABLE video_pub_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN video_pub::VideoSub
  EXECUTABLE video_sub_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN video_pub::ImgPub
  EXECUTABLE img_pub_node
)

## Install
ament_auto_package(
  INSTALL_TO_SHARE
  video
  config
)
