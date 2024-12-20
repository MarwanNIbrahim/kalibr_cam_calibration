# Init CMake (require at least version 3.16)
cmake_minimum_required(VERSION 3.16)

# Find Python installation and its NumPy support. Execute before pybind11 to
# prime CMake caches.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# External pybind11 checked-out as a submodule
add_subdirectory(external/pybind11 EXCLUDE_FROM_ALL)

pybind11_add_module(
  kalibr_cam_calibration MODULE src/kalibr_cam_calibration.cpp
  external/pybind11_opencv_numpy/ndarray_converter.cpp)

target_link_libraries(
  kalibr_cam_calibration
  PRIVATE kalibr::cam_calibration
          ${OpenCV_LIBS}
          aslam_backend
          aslam_cameras
          aslam_cameras_april
          aslam_splines
          bsplines
          incremental_calibration
          kalibr_errorterms
          sm_boost
          sm_kinematics
          Python3::Module
          Python3::NumPy)
target_include_directories(
  kalibr_cam_calibration
  PRIVATE ${PROJECT_SOURCE_DIR}/include/kalibr_cam_calibration)