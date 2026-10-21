#pragma once
#include "kalibr_imu_camera_calibration/iccCalibrator.hpp"
#include "utilities/kalibr_backend/calibrator_base.hpp"

#include <boost/property_tree/json_parser.hpp>

#include <algorithm>
#include <filesystem>

namespace kalibr::backend {
using ImuParameters = ::ImuParameters;

/**
 * Subset of the IccCalibrator::buildProblem() settings.
 */
struct ImuCameraOptions {
    size_t splineOrder        = 6;
    size_t poseKnotsPerSecond = 70;
    size_t biasKnotsPerSecond = 70;
    bool estimateTimeShift    = true;
    int maxIterations         = 30;
    double timeOffsetPadding  = 0.02;  // seconds
    double huberAccel         = -1.0;  // <= 0 disables the Huber loss
    double huberGyro          = -1.0;  // <= 0 disables the Huber loss
    bool verbose              = false;
};

struct ImuCameraResult {
    Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();  // IMU frame -> camera frame
    double timeShift          = 0.0;  // seconds, t_imu = t_cam + timeShift
    bool converged            = false;
    double reprojectionError  = 0.0;  // px, mean
    double gyroscopeError     = 0.0;  // rad/s, mean
    double accelerometerError = 0.0;  // m/s^2, mean

    void writeToFile(const std::filesystem::path& filePath) const {
        boost::property_tree::ptree ptree;

        boost::property_tree::ptree rows;
        for (int r = 0; r < 4; ++r) {
            boost::property_tree::ptree row;
            for (int c = 0; c < 4; ++c) {
                boost::property_tree::ptree entry;
                entry.put("", T_cam_imu(r, c));
                row.push_back(std::make_pair("", entry));
            }
            rows.push_back(std::make_pair("", row));
        }
        ptree.add_child("T_cam_imu", rows);
        ptree.put("timeShift", timeShift);
        ptree.put("converged", converged);
        ptree.put("reprojectionError", reprojectionError);
        ptree.put("gyroscopeError", gyroscopeError);
        ptree.put("accelerometerError", accelerometerError);

        boost::property_tree::write_json(filePath, ptree);
    }
};

/**
 * Camera <-> IMU extrinsic and time-offset calibration for a single camera and IMU.
 * Camera intrinsics are taken from a prior monocular calibration and kept fixed.
 * All input timestamps are in microseconds and must come from the same clock.
 */
class ImuCameraCalibrator {
  public:
    ImuCameraCalibrator(const CalibrationResult& camera,
                        const ImuParameters& imu,
                        const ImuCameraOptions& options = ImuCameraOptions())
        : camera(camera), imuParameters(imu), options(options) {
    }

    void addObservation(const aslam::cameras::GridCalibrationTargetObservation& obs) {
        if (obs.hasSuccessfulObservation()) {
            observations.emplace(toMicroseconds(obs.time()), obs);
        }
    }

    void addImuMeasurement(const int64_t timestampUs,
                           const Eigen::Vector3d& gyro,
                           const Eigen::Vector3d& acc) {
        // Measurement covariances from the discrete-time noise, as Kalibr's IccImu.loadImuData
        const double gyroSigma  = std::get<0>(imuParameters.getGyroStatistics());
        const double accelSigma = std::get<0>(imuParameters.getAccelerometerStatistics());
        imuData.emplace_back(static_cast<double>(timestampUs) * 1e-6,
                             gyro,
                             acc,
                             Eigen::Matrix3d::Identity() * gyroSigma * gyroSigma,
                             Eigen::Matrix3d::Identity() * accelSigma * accelSigma);
    }

    /**
     * Bulk add, one row per sample: [t_us, gx, gy, gz, ax, ay, az].
     */
    void addImuMeasurements(const Eigen::Ref<const Eigen::MatrixXd>& data) {
        if (data.cols() != 7) {
            throw std::invalid_argument("IMU data must have 7 columns: t_us, gx, gy, gz, ax, ay, az");
        }
        imuData.reserve(imuData.size() + static_cast<size_t>(data.rows()));
        for (Eigen::Index i = 0; i < data.rows(); ++i) {
            addImuMeasurement(static_cast<int64_t>(std::llround(data(i, 0))),
                              data.block<1, 3>(i, 1).transpose(),
                              data.block<1, 3>(i, 4).transpose());
        }
    }

    size_t getNumValidObservations() const {
        return observations.size();
    }

    size_t getNumImuMeasurements() const {
        return imuData.size();
    }

    ImuCameraResult calibrate() {
        if (observations.size() < 2) {
            throw std::runtime_error("Not enough successful target observations");
        }
        if (imuData.size() < 2) {
            throw std::runtime_error("Not enough IMU measurements");
        }

        if (camera.distortionModel == DistortionModel::EQUIDISTANT) {
            return calibrateImpl<aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
                                 aslam::cameras::EquidistantDistortion>();
        }
        return calibrateImpl<aslam::cameras::DistortedPinholeCameraGeometry,
                             aslam::cameras::RadialTangentialDistortion>();
    }

  private:
    template <typename CameraGeometryType, typename DistortionType>
    ImuCameraResult calibrateImpl() {
        const std::vector<double> intrinsics = {camera.focalLength.x,
                                                camera.focalLength.y,
                                                camera.principalPoint.x,
                                                camera.principalPoint.y};

        auto observationsMap = boost::make_shared<
            std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>>(observations);
        auto iccCamera = boost::make_shared<IccCamera<CameraGeometryType, DistortionType>>(
            intrinsics, camera.distortionCoefficients, camera.imageResolution, observationsMap);

        // The detector only had a dummy geometry, so re-estimate target poses with the
        // calibrated intrinsics. The pose spline is initialized from these.
        const auto geometry = iccCamera->getCameraGeometry();
        for (auto it = observationsMap->begin(); it != observationsMap->end();) {
            sm::kinematics::Transformation T_t_c;
            if (geometry->estimateTransformation(it->second, T_t_c)) {
                it->second.set_T_t_c(T_t_c);
                ++it;
            } else {
                it = observationsMap->erase(it);
            }
        }
        if (observationsMap->size() < 2) {
            throw std::runtime_error("Could not estimate target poses for the observations");
        }

        auto sortedImuData = boost::make_shared<std::vector<IccImuUtils::ImuMeasurement>>(imuData);
        std::sort(sortedImuData->begin(), sortedImuData->end(), [](const auto& a, const auto& b) {
            return a.stamp < b.stamp;
        });
        auto iccImu = boost::make_shared<IccImu>(imuParameters, sortedImuData);

        IccCalibrator<CameraGeometryType, DistortionType> calibrator(iccCamera, iccImu);
        calibrator.buildProblem(options.splineOrder,
                                options.poseKnotsPerSecond,
                                options.biasKnotsPerSecond,
                                1e6,
                                1e5,
                                true,
                                -1,
                                options.huberAccel,
                                options.huberGyro,
                                !options.estimateTimeShift,
                                true,
                                options.maxIterations,
                                1.0,
                                1.0,
                                options.timeOffsetPadding,
                                options.verbose);
        calibrator.optimize(nullptr, static_cast<size_t>(options.maxIterations), options.verbose);

        if (options.verbose) {
            calibrator.printErrorStatistics(std::cout);
        }

        const auto icc = calibrator.getResult();
        ImuCameraResult result;
        result.T_cam_imu          = icc.T_cam_imu;
        result.timeShift          = icc.t_cam_imu;
        result.converged          = icc.converged;
        result.reprojectionError  = icc.error_info.meanReprojectionError;
        result.gyroscopeError     = icc.error_info.meanGyroscopeError;
        result.accelerometerError = icc.error_info.meanAccelerometerError;
        return result;
    }

    CalibrationResult camera;
    ImuParameters imuParameters;
    ImuCameraOptions options;
    std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation> observations;
    std::vector<IccImuUtils::ImuMeasurement> imuData;
};
}  // namespace kalibr::backend
