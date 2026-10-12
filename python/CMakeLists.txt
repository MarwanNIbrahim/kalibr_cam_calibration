# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

# Find Python installation and its NumPy support. Execute before pybind11 to
# prime CMake caches.
FIND_PACKAGE(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# External pybind11 checked-out as a submodule
ADD_SUBDIRECTORY(external/pybind11 EXCLUDE_FROM_ALL)

PYBIND11_ADD_MODULE(
        kalibr_cam_calibration MODULE src/kalibr_cam_calibration.cpp
        external/pybind11_opencv_numpy/ndarray_converter.cpp)

TARGET_LINK_LIBRARIES(
        kalibr_cam_calibration
        ${OpenCV_LIBS}
        PRIVATE kalibr::cam_calibration Python3::NumPy)

# The kalibr shared libraries are installed next to the module (see thirdparty/CMakeLists.txt)
SET_TARGET_PROPERTIES(kalibr_cam_calibration PROPERTIES INSTALL_RPATH "$ORIGIN")
INSTALL(TARGETS kalibr_cam_calibration LIBRARY DESTINATION ${KALIBR_LIB_DESTINATION})