#pragma once

#include "kalibr_imu_camera_calibration/iccCamera.hpp"
#include "utilities/kalibr_frontend/pattern_options.hpp"

#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <atomic>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>

namespace kalibr::frontend {

/**
 * Hold image and corresponding timestamp in microseconds.
 */
struct StampedImage {
    cv::Mat image;
    int64_t timestamp;  // microseconds

    StampedImage(){};

    StampedImage(cv::Mat img, const int64_t ts) : image(std::move(img)), timestamp(ts){};

    /**
     * Clone the underlying image.
     *
     * @return
     */
    StampedImage clone() const {
        StampedImage clone;
        clone.image     = image.clone();
        clone.timestamp = timestamp;
        return clone;
    }
};

class MarkerDetector {
  public:
    MarkerDetector(const cv::Size& imageSize, const PatternOptions& patternOptions) {
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid{nullptr};

        switch (patternOptions.patternType) {
            case PatternType::CHESSBOARD: {
                aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions options;
                options.filterQuads          = true;
                options.normalizeImage       = true;
                options.useAdaptiveThreshold = true;
                options.performFastCheck     = false;
                options.windowWidth          = 5;
                options.showExtractionVideo  = false;

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
                    patternOptions.patternSize.width,
                    patternOptions.patternSize.height,
                    patternOptions.markerSize,
                    patternOptions.markerSize,
                    options);
                break;
            }
            case PatternType::ASYMMETRIC_CIRCLES_GRID: {
                aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
                options.showExtractionVideo     = false;
                options.useAsymmetricCirclegrid = true;

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(
                    patternOptions.patternSize.width,
                    patternOptions.patternSize.height,
                    patternOptions.markerSize,
                    options);
                break;
            }
            case PatternType::APRIL_GRID: {
                aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
                // enforce more than one row --> pnp solution can be bad if all points are almost on
                // a line...
                options.minTagsForValidObs =
                    std::max(patternOptions.patternSize.height, patternOptions.patternSize.width) +
                    1;
                options.showExtractionVideo = false;

                grid = boost::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
                    patternOptions.patternSize.width,
                    patternOptions.patternSize.height,
                    patternOptions.markerSize,
                    patternOptions.markerSpacing / patternOptions.markerSize,
                    options);
                break;
            }
            default:
                throw std::runtime_error("Not implemented calibration pattern");
        }

        aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
        detectorOptions.imageStepping          = false;
        detectorOptions.plotCornerReprojection = false;
        detectorOptions.filterCornerOutliers   = true;

        auto targetObservations = boost::make_shared<
            std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();

        // Initialize a dummy camera geometry instance
        std::vector<double> intrinsics = {static_cast<double>(imageSize.width),
                                          static_cast<double>(imageSize.width),
                                          static_cast<double>(imageSize.width / 2),
                                          static_cast<double>(imageSize.height / 2)};
        std::vector<double> distCoeffs = {0.0, 0.0, 0.0, 0.0};

        const auto iccCamera =
            boost::make_shared<IccCamera<aslam::cameras::DistortedPinholeCameraGeometry,
                                         aslam::cameras::RadialTangentialDistortion>>(
                intrinsics, distCoeffs, imageSize, targetObservations);

        detector = boost::make_shared<aslam::cameras::GridDetector>(
            iccCamera->getCameraGeometry(), grid, detectorOptions);
    }

    aslam::cameras::GridCalibrationTargetObservation detectPattern(
        const StampedImage& stampedImage) {
        if (stampedImage.image.channels() != 1) {
            throw std::runtime_error("Calibrator expects grayscale images!");
        }

        aslam::cameras::GridCalibrationTargetObservation observation;
        const aslam::Time stamp(static_cast<double>(stampedImage.timestamp) * 1e-6);
        detector->findTarget(stampedImage.image, stamp, observation);
        observation.setImage(stampedImage.image);

        return observation;
    }

    static void DrawObservation(
        cv::Mat& image, const aslam::cameras::GridCalibrationTargetObservation& observation) {
        // Colors used in visualization of detected calibtion pattern
        static const std::vector<cv::Scalar> colors{
            cv::Scalar(255, 0, 0),   cv::Scalar(128, 0, 0),   cv::Scalar(0, 255, 0),
            cv::Scalar(0, 128, 0),   cv::Scalar(0, 0, 255),   cv::Scalar(0, 0, 128),
            cv::Scalar(255, 255, 0), cv::Scalar(128, 255, 0), cv::Scalar(0, 255, 255),
            cv::Scalar(0, 255, 128), cv::Scalar(255, 128, 0), cv::Scalar(128, 128, 0),
            cv::Scalar(0, 128, 255), cv::Scalar(0, 128, 128), cv::Scalar(255, 0, 255),
            cv::Scalar(128, 0, 255), cv::Scalar(0, 255, 255), cv::Scalar(0, 128, 255),
            cv::Scalar(255, 0, 128), cv::Scalar(128, 0, 128), cv::Scalar(0, 255, 128),
            cv::Scalar(0, 128, 128)};

        cv::Point prevPoint(-1, -1);
        for (size_t y = 0; y < observation.target()->rows(); ++y) {
            const auto color = colors[y % colors.size()];
            for (size_t x = 0; x < observation.target()->cols(); ++x) {
                const auto idx = y * observation.target()->cols() + x;
                Eigen::Vector2d point;
                if (observation.imagePoint(idx, point)) {
                    const cv::Point cvPoint(static_cast<int>(point.x()),
                                            static_cast<int>(point.y()));
                    cv::circle(image, cvPoint, 8, color, 2);
                    if (prevPoint != cv::Point(-1, -1)) {
                        cv::line(image, prevPoint, cvPoint, color, 2);
                    }
                    prevPoint = cvPoint;
                }
            }
        }
    }

  private:
    boost::shared_ptr<aslam::cameras::GridDetector> detector{nullptr};
};
}  // namespace kalibr::frontend
