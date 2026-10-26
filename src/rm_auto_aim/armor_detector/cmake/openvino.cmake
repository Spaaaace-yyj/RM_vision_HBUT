# Use the Ubuntu C++ SDK (archive/APT), not the Linux PyPI wheel's old libstdc++ ABI.
option(ARMOR_ENABLE_OPENVINO "Build OpenVINO CPU inference backend" ON)
if(ARMOR_ENABLE_OPENVINO)
  # OpenVINOConfig-version.cmake only accepts requests from its own major year.
  # Discover first, then check the minimum numerically so a 2025 SDK can satisfy
  # our 2024.4-or-newer source API requirement.
  find_package(OpenVINO QUIET COMPONENTS Runtime)
  if(OpenVINO_FOUND AND OpenVINO_VERSION VERSION_LESS "2024.4")
    message(FATAL_ERROR "OpenVINO >= 2024.4 is required; found ${OpenVINO_VERSION}. Set ARMOR_ENABLE_OPENVINO=OFF to build without this backend.")
  endif()
  if(OpenVINO_FOUND AND DEFINED OpenVINO_GLIBCXX_USE_CXX11_ABI AND
      OpenVINO_GLIBCXX_USE_CXX11_ABI STREQUAL "0")
    message(FATAL_ERROR "OpenVINO uses old C++ ABI=0, incompatible with Ubuntu OpenCV/ROS. Use the Ubuntu archive/APT SDK and set OpenVINO_DIR to its runtime/cmake. See OPENVINO.md.")
  endif()
endif()
