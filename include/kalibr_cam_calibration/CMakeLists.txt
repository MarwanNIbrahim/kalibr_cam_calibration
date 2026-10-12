FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(fmt REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

ADD_LIBRARY(cam_calibration INTERFACE)
ADD_LIBRARY(kalibr::cam_calibration ALIAS cam_calibration)

TARGET_LINK_LIBRARIES(
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

TARGET_INCLUDE_DIRECTORIES(
  cam_calibration
  INTERFACE ${PROJECT_SOURCE_DIR}/include/kalibr_cam_calibration)
