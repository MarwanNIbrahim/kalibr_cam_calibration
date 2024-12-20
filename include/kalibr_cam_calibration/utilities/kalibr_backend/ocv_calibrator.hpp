#pragma once
#include <opencv2/calib3d.hpp>
#include "utilities/kalibr_backend/calibrator_base.hpp"

namespace kalibr::backend {
class OCVCalibrator : public CalibratorBase {
  public:
    OCVCalibrator(const DistortionModel distModel, const std::optional<CalibrationResult> &initGuess = std::nullopt, 
    const CalibrationFlags flags = CalibrationFlags::NONE) : CalibratorBase(distModel, initGuess, flags) {
    }

    virtual CalibrationResult calibrateCamera() override {
        // First, extract points from observations
        std::vector<std::vector<cv::Point3f>> targetPoints;
        std::vector<std::vector<cv::Point2f>> imagePoints;
        for (const auto& obs : observations) {
            if (!obs.hasSuccessfulObservation()) {
                continue;
            }
            std::vector<cv::Point3f> targetCorners;
            obs.getCornersTargetFrame(targetCorners);
            targetPoints.push_back(targetCorners);

            std::vector<cv::Point2f> imageCorners;
            obs.getCornersImageFrame(imageCorners);
            imagePoints.push_back(imageCorners);
        }

        cv::Mat K, D;
        if(initialGuess.has_value()){
            K = cv::Mat::zeros(3, 3, CV_64F);
            D = cv::Mat::zeros(1, 4, CV_64F);
            K.at<double>(0, 0) = initialGuess->focalLength.x;
            K.at<double>(1, 1) = initialGuess->focalLength.y;

            K.at<double>(0, 2) = initialGuess->principalPoint.x;
            K.at<double>(1, 2) = initialGuess->principalPoint.y;

            size_t i=0;
            for(const double distCoeff : initialGuess->distortionCoefficients){
                D.at<double>(i) = distCoeff;
                ++i;
            }
        }

        std::vector<cv::Mat> rvecs, tvecs;
        CalibrationResult result;
        if (distortionModel == DistortionModel::EQUIDISTANT) {
            int cvFlags = 0;
            if(initialGuess.has_value()){
                cvFlags |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
                if(calibFlags & 1){
                    cvFlags |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT | (1<<11); //Fix focal length not in namespace
                }
                if(calibFlags & 2){
                    cvFlags |= cv::fisheye::CALIB_FIX_K1 | cv::fisheye::CALIB_FIX_K2 | cv::fisheye::CALIB_FIX_K3 | cv::fisheye::CALIB_FIX_K4;
                }
            }
            result.reprojectionError =
                cv::fisheye::calibrate(targetPoints, imagePoints, resolution, K, D, rvecs, tvecs, cvFlags);
        } else {
            int cvFlags = 0;
            if(initialGuess.has_value()){
                cvFlags |= cv::CALIB_USE_INTRINSIC_GUESS;
                if(calibFlags & 1){
                    cvFlags |= cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_FOCAL_LENGTH;
                }
                if(calibFlags & 2){
                    cvFlags |= cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4;
                }
            }
            result.reprojectionError =
                cv::calibrateCamera(targetPoints, imagePoints, resolution, K, D, rvecs, tvecs, cvFlags);
        }

        result.distortionModel = distortionModel;
        result.focalLength.x   = K.at<double>(0, 0);
        result.focalLength.y   = K.at<double>(1, 1);

        result.principalPoint.x = K.at<double>(0, 2);
        result.principalPoint.y = K.at<double>(1, 2);

        for (size_t i = 0; i < D.size().area(); ++i) {
            result.distortionCoefficients.push_back(D.at<double>(i));
        }

        result.imageResolution = resolution;

        return result;
    }
};
}  // namespace kalibr::backend
