add_executable(sample_run_detector sample_run_detector.cpp)
target_link_libraries(
  sample_run_detector
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
          CLI11::CLI11)

target_include_directories(
  sample_run_detector
  PRIVATE ${PROJECT_SOURCE_DIR}/include/kalibr_cam_calibration)
