# Use the Ubuntu C++ SDK (archive/APT), not the Linux PyPI wheel's old libstdc++ ABI.
option(ARMOR_ENABLE_OPENVINO "Build OpenVINO CPU inference backend" ON)
if(ARMOR_ENABLE_OPENVINO)
  find_package(OpenVINO 2024.4 QUIET COMPONENTS Runtime)
  if(OpenVINO_FOUND AND DEFINED OpenVINO_GLIBCXX_USE_CXX11_ABI AND
      OpenVINO_GLIBCXX_USE_CXX11_ABI STREQUAL "0")
    message(FATAL_ERROR "OpenVINO uses old C++ ABI=0, incompatible with Ubuntu OpenCV/ROS. Use the Ubuntu archive/APT SDK and set OpenVINO_DIR to its runtime/cmake. See OPENVINO.md.")
  endif()
endif()
