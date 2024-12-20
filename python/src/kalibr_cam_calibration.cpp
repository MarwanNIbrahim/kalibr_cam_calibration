#include "../external/pybind11_opencv_numpy/ndarray_converter.h"
#include "utilities/kalibr_backend/kalibr_calibrator.hpp"
#include "utilities/kalibr_backend/ocv_calibrator.hpp"
#include "utilities/kalibr_frontend/marker_detector.hpp"

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py = pybind11;

namespace pybind11::detail {

template <>
struct type_caster<cv::Size> {
    PYBIND11_TYPE_CASTER(cv::Size, _("tuple_xy"));

    bool load(handle obj, bool) {
        if (!py::isinstance<py::tuple>(obj)) {
            std::logic_error("Size(width,height) should be a tuple!");
            return false;
        }

        auto pt = reinterpret_borrow<py::tuple>(obj);
        if (pt.size() != 2) {
            std::logic_error("Size(width,height) tuple should be size of 2");
            return false;
        }

        value = cv::Size(pt[0].cast<int>(), pt[1].cast<int>());
        return true;
    }

    static handle cast(const cv::Size& resolution, return_value_policy, handle) {
        return py::make_tuple(resolution.width, resolution.height).release();
    }
};

template <>
struct type_caster<cv::Point2f> {
    PYBIND11_TYPE_CASTER(cv::Point2f, _("tuple_xy"));

    bool load(handle obj, bool) {
        if (!py::isinstance<py::tuple>(obj)) {
            std::logic_error("Point2f(x,y) should be a tuple!");
            return false;
        }

        auto pt = reinterpret_borrow<py::tuple>(obj);
        if (pt.size() != 2) {
            std::logic_error("Point2f(x,y) tuple should be size of 2");
            return false;
        }

        value = cv::Point2f(pt[0].cast<float>(), pt[1].cast<float>());
        return true;
    }

    static handle cast(const cv::Point2f& point, return_value_policy, handle) {
        return py::make_tuple(point.x, point.y).release();
    }
};

template <>
struct type_caster<cv::Point3f> {
    PYBIND11_TYPE_CASTER(cv::Point3f, _("tuple_xyz"));

    bool load(handle obj, bool) {
        if (!py::isinstance<py::tuple>(obj)) {
            std::logic_error("Point3f(x,y,z) should be a tuple!");
            return false;
        }

        auto pt = reinterpret_borrow<py::tuple>(obj);
        if (pt.size() != 3) {
            std::logic_error("Point3f(x,y,z) tuple should be size of 3");
            return false;
        }

        value = cv::Point3f(pt[0].cast<float>(), pt[1].cast<float>(), pt[2].cast<float>());
        return true;
    }

    static handle cast(const cv::Point3f& point, return_value_policy, handle) {
        return py::make_tuple(point.x, point.y, point.z).release();
    }
};

template <>
struct type_caster<cv::Point2d> {
    PYBIND11_TYPE_CASTER(cv::Point2d, _("tuple_xy"));

    bool load(handle obj, bool) {
        if (!py::isinstance<py::tuple>(obj)) {
            std::logic_error("Point2d(x,y) should be a tuple!");
            return false;
        }

        auto pt = reinterpret_borrow<py::tuple>(obj);
        if (pt.size() != 2) {
            std::logic_error("Point2d(x,y) tuple should be size of 2");
            return false;
        }

        value = cv::Point2d(pt[0].cast<double>(), pt[1].cast<double>());
        return true;
    }

    static handle cast(const cv::Point2d& point, return_value_policy, handle) {
        return py::make_tuple(point.x, point.y).release();
    }
};
}  // namespace pybind11::detail

PYBIND11_MODULE(kalibr_cam_calibration, m) {
    using pybind11::operator""_a;
    m.attr("__version__") = "1.0.0";

    NDArrayConverter::init_numpy();

    // Ensure we can convert back and forth from filesystem path to string
    py::class_<std::filesystem::path>(m, "Path").def(py::init<std::string>(), py::arg("path"));
    py::implicitly_convertible<std::string, std::filesystem::path>();

    // Define frontend module
    auto m_frontend = m.def_submodule("frontend");
    py::enum_<kalibr::frontend::PatternType>(m_frontend, "PatternType")
        .value("CHESSBOARD", kalibr::frontend::PatternType::CHESSBOARD)
        .value("ASYMMETRIC_CIRCLES_GRID", kalibr::frontend::PatternType::ASYMMETRIC_CIRCLES_GRID)
        .value("APRIL_GRID", kalibr::frontend::PatternType::APRIL_GRID)
        .export_values();

    py::class_<kalibr::frontend::PatternOptions>(m_frontend, "PatternOptions")
        .def(py::init<>(), "Default constructor for PatternOptions struct")
        .def_readwrite("patternType",
                       &kalibr::frontend::PatternOptions::patternType,
                       "Type of calibration pattern used")
        .def_readwrite(
            "patternSize",
            &kalibr::frontend::PatternOptions::patternSize,
            "Size of calibration pattern used following OpenCV convention: [numRows, numCols]")
        .def_readwrite("markerSize",
                       &kalibr::frontend::PatternOptions::markerSize,
                       "Size of calibration marker used in physical units (i.e: meters)")
        .def_readwrite("markerSpacing",
                       &kalibr::frontend::PatternOptions::markerSpacing,
                       "Calibration marker spacing in physical units (i.e: meters). Only relevant "
                       "for April Grid");

    py::class_<kalibr::frontend::StampedImage>(m_frontend, "StampedImage")
        .def(py::init<>(), "Default constructor for StampedImage struct")
        .def_readwrite("image", &kalibr::frontend::StampedImage::image, "Image contents")
        .def_readwrite("timestamp",
                       &kalibr::frontend::StampedImage::timestamp,
                       "Capture time of the image, usually refers to start of exposure")
        .def("clone", &kalibr::frontend::StampedImage::clone, "Clone the current image");

    py::class_<aslam::cameras::GridCalibrationTargetObservation>(m_frontend,
                                                                 "GridCalibrationTargetObservation")
        .def("getCornersTargetFrame",
             [](const aslam::cameras::GridCalibrationTargetObservation& self) {
                 std::vector<cv::Point3f> corners;
                 self.getCornersTargetFrame(corners);
                 return corners;
             })
        .def("getCornersImageFrame",
             [](const aslam::cameras::GridCalibrationTargetObservation& self) {
                 std::vector<cv::Point2f> corners;
                 self.getCornersImageFrame(corners);
                 return corners;
             })
        .def("getImage",
             [](const aslam::cameras::GridCalibrationTargetObservation& self) {
                 return self.image();
             })
        .def("hasSuccessfulObservation",
             &aslam::cameras::GridCalibrationTargetObservation::hasSuccessfulObservation,
             "Check if the current observation was successful");

    py::class_<kalibr::frontend::MarkerDetector>(m_frontend, "MarkerDetector")
        .def(py::init<const cv::Size&, const kalibr::frontend::PatternOptions&>(),
             "imageSize"_a,
             "patternOptions"_a,
             "Initialize marker detector to detect specified marker type in images with given "
             "resolution")
        .def("detectPattern",
             &kalibr::frontend::MarkerDetector::detectPattern,
             "stampedImage"_a,
             "Detect calibration markers in given image.")
        .def_static(
            "DrawObservation",
            [](cv::Mat& image, const aslam::cameras::GridCalibrationTargetObservation& obs) {
                kalibr::frontend::MarkerDetector::DrawObservation(image, obs);
            });

    // Define backend module
    auto m_backend = m.def_submodule("backend");
    py::enum_<kalibr::backend::DistortionModel>(m_backend, "DistortionModel")
        .value("RADIAL_TANGENTIAL", kalibr::backend::DistortionModel::RADIAL_TANGENTIAL)
        .value("EQUIDISTANT", kalibr::backend::DistortionModel::EQUIDISTANT)
        .export_values();

    py::enum_<kalibr::backend::CalibrationFlags>(m_backend, "CalibrationFlags")
        .value("NONE", kalibr::backend::CalibrationFlags::NONE)
        .value("FIX_DISTORTION", kalibr::backend::CalibrationFlags::FIX_DISTORTION)
        .value("FIX_INTRINSICS", kalibr::backend::CalibrationFlags::FIX_INTRINSICS)
        .export_values();

    py::class_<kalibr::backend::CalibrationResult>(m_backend, "CalibrationResult")
        .def(py::init<>(), "Default initializer for CalibrationResult struct")
        .def_readwrite("imageResolution",
                       &kalibr::backend::CalibrationResult::imageResolution,
                       "Image resolution")
        .def_readwrite(
            "focalLength", &kalibr::backend::CalibrationResult::focalLength, "Focal length")
        .def_readwrite("principalPoint",
                       &kalibr::backend::CalibrationResult::principalPoint,
                       "Principal point")
        .def_readwrite("distortionCoefficients",
                       &kalibr::backend::CalibrationResult::distortionCoefficients,
                       "Distortion coefficients")
        .def_readwrite("distortionModel",
                       &kalibr::backend::CalibrationResult::distortionModel,
                       "Distortion model")
        .def_readwrite("patternOptions",
                       &kalibr::backend::CalibrationResult::patternOptions,
                       "Optional calibration pattern settings that can be added to result")
        .def_readwrite("cameraName",
                       &kalibr::backend::CalibrationResult::cameraName,
                       "Optional camera name that can be added to result")
        .def_readwrite("reprojectionError",
                       &kalibr::backend::CalibrationResult::reprojectionError,
                       "Reprojection error for calibration result")
        .def("writeToFile",
             &kalibr::backend::CalibrationResult::writeToFile,
             "filePath"_a,
             "Save the current calibration result to a file.")
        .def_static("LoadFromFile", [](const std::filesystem::path& filePath) {
            return kalibr::backend::CalibrationResult::LoadFromFile(filePath);
        });

    py::class_<kalibr::backend::KalibrCalibrator>(m_backend, "KalibrCalibrator")
        .def(py::init<const kalibr::backend::DistortionModel,
                      const std::optional<kalibr::backend::CalibrationResult>&,
                      const kalibr::backend::CalibrationFlags>(),
             "distortionModel"_a,
             "initGuess"_a,
             "flags"_a,
             "Initialize Kalibr-based monocular camera calibrator")
        .def("getNumValidObservations",
             &kalibr::backend::KalibrCalibrator::getNumValidObservations,
             "Get number of images with at least one observed marker")
        .def("addObservation",
             [](kalibr::backend::KalibrCalibrator& self,
                const aslam::cameras::GridCalibrationTargetObservation& obs) {
                 self.addObservation(obs);
             })
        .def("calibrateCamera",
             &kalibr::backend::KalibrCalibrator::calibrateCamera,
             "Run camera calibration based on the added observations.");

    py::class_<kalibr::backend::OCVCalibrator>(m_backend, "OCVCalibrator")
        .def(py::init<const kalibr::backend::DistortionModel,
                      const std::optional<kalibr::backend::CalibrationResult>&,
                      const kalibr::backend::CalibrationFlags>(),
             "distortionModel"_a,
             "initGuess"_a,
             "flags"_a,
             "Initialize OpenCV-based monocular camera calibrator")
        .def("getNumValidObservations",
             &kalibr::backend::OCVCalibrator::getNumValidObservations,
             "Get number of images with at least one observed marker")
        .def("addObservation",
             [](kalibr::backend::OCVCalibrator& self,
                const aslam::cameras::GridCalibrationTargetObservation& obs) {
                 self.addObservation(obs);
             })
        .def("calibrateCamera",
             &kalibr::backend::OCVCalibrator::calibrateCamera,
             "Run camera calibration based on the added observations.");
}
