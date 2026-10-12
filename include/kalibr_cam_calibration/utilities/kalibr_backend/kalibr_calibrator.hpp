#pragma once
#include "kalibr_calibrate_cameras/CameraCalibrator.hpp"
#include "utilities/kalibr_backend/calibrator_base.hpp"

namespace kalibr::backend {
class KalibrCalibrator : public CalibratorBase {
  public:
    KalibrCalibrator(const DistortionModel distModel, const std::optional<CalibrationResult> &initGuess = std::nullopt, 
    const CalibrationFlags flags = CalibrationFlags::NONE) : CalibratorBase(distModel, initGuess, flags) {
    }

    virtual CalibrationResult calibrateCamera() override {
        IccCameraUtils::ObservationsPtr observationsMap = boost::make_shared<
            std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>();
        boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid = nullptr;
        for (auto& obs : observations) {
            observationsMap->emplace(toMicroseconds(obs.time()), obs);
            if (grid == nullptr) {
                grid = obs.target();
            }
        }

        std::vector<double> intrinsics = {static_cast<double>(resolution.width),
                                          static_cast<double>(resolution.width),
                                          static_cast<double>(resolution.width / 2),
                                          static_cast<double>(resolution.height / 2)};
        std::vector<double> distCoeffs = {0.0, 0.0, 0.0, 0.0};
        const bool useIntrinsicGuess = initialGuess.has_value();
        if(useIntrinsicGuess){
            intrinsics = {initialGuess->focalLength.x, initialGuess->focalLength.y, initialGuess->principalPoint.x, initialGuess->principalPoint.y};
        
            size_t i=0;
            for(const double distCoeff : initialGuess->distortionCoefficients){
                distCoeffs[i] = distCoeff;
                ++i;
            }
        }

        CameraCalibrationUtils::CalibrationResult initialResults;
        const bool distortionActive = !(calibFlags & 1);
        const bool intrinsicsActive = !(calibFlags & 2);
            
        if (distortionModel == DistortionModel::EQUIDISTANT) {
            using CameraGeometryType = aslam::cameras::EquidistantDistortedPinholeCameraGeometry;
            using DistortionType     = aslam::cameras::EquidistantDistortion;
            const auto iccCamera =
                boost::make_shared<IccCamera<CameraGeometryType, DistortionType>>(
                    intrinsics, distCoeffs, resolution, observationsMap);
            auto cameraGeometry =
                boost::make_shared<CameraGeometry<CameraGeometryType, DistortionType>>(iccCamera);
            const bool success =
                cameraGeometry->initGeometryFromObservations(observationsMap, grid, useIntrinsicGuess, distortionActive, intrinsicsActive);
            initialResults = cameraGeometry->getResult();
        } else {
            using CameraGeometryType = aslam::cameras::DistortedPinholeCameraGeometry;
            using DistortionType     = aslam::cameras::RadialTangentialDistortion;
            const auto iccCamera =
                boost::make_shared<IccCamera<CameraGeometryType, DistortionType>>(
                    intrinsics, distCoeffs, resolution, observationsMap);
            auto cameraGeometry =
                boost::make_shared<CameraGeometry<CameraGeometryType, DistortionType>>(iccCamera);
            const bool success =
                cameraGeometry->initGeometryFromObservations(observationsMap, grid, useIntrinsicGuess, distortionActive, intrinsicsActive);
            initialResults = cameraGeometry->getResult();
        }

        CalibrationResult result;

        result.distortionModel = distortionModel;
        result.focalLength.x   = initialResults.projection.at(0);
        result.focalLength.y   = initialResults.projection.at(1);

        result.principalPoint.x       = initialResults.projection.at(2);
        result.principalPoint.y       = initialResults.projection.at(3);
        result.distortionCoefficients = initialResults.distortion;
        result.reprojectionError =
            (initialResults.err_info.mean.x() + initialResults.err_info.mean.y()) / 2.0;
        result.imageResolution = resolution;
        return result;
    }
};
}  // namespace kalibr::backend
