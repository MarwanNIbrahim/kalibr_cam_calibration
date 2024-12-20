find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)

add_library(cam_calibration INTERFACE)
add_library(kalibr::cam_calibration ALIAS cam_calibration)

target_link_libraries(
  cam_calibration
  INTERFACE ${SYSTEM_THREAD_LIBS}
            Eigen3::Eigen
            fmt::fmt
            aslam_backend
            aslam_cameras
            aslam_cv_backend
            aslam_cameras_april
            aslam_splines
            bsplines
            incremental_calibration
            kalibr_errorterms
            sm_kinematics
            aslam_cv_error_terms
            sm_boost)
target_include_directories(
  cam_calibration
  INTERFACE ${PROJECT_SOURCE_DIR}/include/kalibr_cam_calibration)
