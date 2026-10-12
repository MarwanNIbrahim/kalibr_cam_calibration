#pragma once
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include "utilities/kalibr_backend/calibration_result.hpp"

namespace kalibr::backend {
// Observation key: aslam::Time holds seconds, the public API uses microseconds
inline int64_t toMicroseconds(const aslam::Time& time) {
    return std::llround(time.toSec() * 1e6);
}

enum CalibrationFlags{
    NONE = 0,
    FIX_DISTORTION = 1,
    FIX_INTRINSICS = 2
};

class CalibratorBase {
  public:
    CalibratorBase(const DistortionModel distModel, const std::optional<CalibrationResult> &initGuess = std::nullopt, 
    const CalibrationFlags flags = CalibrationFlags::NONE) : distortionModel(distModel), initialGuess(initGuess), calibFlags(flags) {
    }

    void addObservation(const aslam::cameras::GridCalibrationTargetObservation& obs) {
        if (!resolutionSet) {
            resolution    = obs.image().size();
            resolutionSet = true;
        }
        if (obs.hasSuccessfulObservation()) {
            ++numValidObs;
        }
        observations.push_back(obs);
    }

    size_t getNumValidObservations() const {
        return numValidObs;
    }

    virtual CalibrationResult calibrateCamera() = 0;

  protected:
    size_t numValidObs{0};
    bool resolutionSet{false};
    cv::Size resolution;
    std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;
    DistortionModel distortionModel;
    std::optional<CalibrationResult> initialGuess;
    CalibrationFlags calibFlags;
};
}  // namespace kalibr::backend
