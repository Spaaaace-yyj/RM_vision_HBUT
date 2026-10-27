cmake_minimum_required(VERSION 3.16)
project(armor_camera_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ARMOR_DIR "${CMAKE_CURRENT_LIST_DIR}/../..")
set(MVSDK_ROOT "${ARMOR_DIR}/../../driver/mindvision_camera/mvsdk" CACHE PATH "MindVision SDK root")
enable_testing()

# Real SDK declarations + fake functions: no camera, ROS, OpenCV or SDK shared library required.
add_executable(test_embedded_camera test_embedded_camera.cpp "${ARMOR_DIR}/src/embedded_camera.cpp")
target_include_directories(test_embedded_camera PRIVATE "${ARMOR_DIR}/include" "${MVSDK_ROOT}/include")
target_compile_definitions(test_embedded_camera PRIVATE ARMOR_DETECTOR_WITH_CAMERA)
target_compile_options(test_embedded_camera PRIVATE -Wall -Wextra -Werror)
add_test(NAME embedded_camera COMMAND test_embedded_camera)

add_executable(test_camera_disabled test_camera_disabled.cpp "${ARMOR_DIR}/src/embedded_camera.cpp")
target_include_directories(test_camera_disabled PRIVATE "${ARMOR_DIR}/include")
target_compile_options(test_camera_disabled PRIVATE -Wall -Wextra -Werror)
add_test(NAME camera_disabled COMMAND test_camera_disabled)
