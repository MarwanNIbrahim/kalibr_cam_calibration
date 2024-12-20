#pragma once
#include <boost/property_tree/json_parser.hpp>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include "utilities/kalibr_frontend/pattern_options.hpp"

namespace kalibr::backend {
enum class DistortionModel { RADIAL_TANGENTIAL, EQUIDISTANT };

struct CalibrationResult {
    static CalibrationResult LoadFromFile(const std::filesystem::path& filePath) {
        if (!std::filesystem::exists(filePath)) {
            throw std::invalid_argument("Could not open file at given path");
        }

        CalibrationResult result;
        boost::property_tree::ptree ptree;
        boost::property_tree::read_json(filePath, ptree);
        boost::optional<std::string> camName = ptree.get_optional<std::string>("name");
        if (camName.has_value()) {
            result.cameraName = camName.value();
        }

        // Get resolution
        auto resolution        = getListFromTree<int>(ptree, "resolution");
        result.imageResolution = cv::Size(resolution[0], resolution[1]);

        // Get principal point
        auto principalPt      = getListFromTree<double>(ptree, "principalPoint");
        result.principalPoint = cv::Point2d(principalPt[0], principalPt[1]);

        // Get focal length
        auto fLength       = getListFromTree<double>(ptree, "focalLength");
        result.focalLength = cv::Point2d(fLength[0], fLength[1]);

        // Get distortion coefficients
        result.distortionCoefficients = getListFromTree<double>(ptree, "distortionCoefficients");
        result.distortionModel =
            (ptree.get<std::string>("distortionModel", "Equidistant") == "Equidistant"
                 ? DistortionModel::EQUIDISTANT
                 : DistortionModel::RADIAL_TANGENTIAL);

        // Get meta data
        boost::property_tree::ptree metaData = ptree.get_child("metaData");
        boost::optional<std::string> patType = metaData.get_optional<std::string>("patternType");
        if (patType.has_value()) {
            result.patternOptions = kalibr::frontend::PatternOptions();
            if (patType.value() == "AprilGrid") {
                result.patternOptions->patternType = kalibr::frontend::PatternType::APRIL_GRID;
            } else if (patType.value() == "Chessboard") {
                result.patternOptions->patternType = kalibr::frontend::PatternType::CHESSBOARD;
            } else {
                result.patternOptions->patternType =
                    kalibr::frontend::PatternType::ASYMMETRIC_CIRCLES_GRID;
            }

            std::vector<int> patternShape      = getListFromTree<int>(metaData, "patternShape");
            result.patternOptions->patternSize = cv::Size(patternShape[0], patternShape[1]);

            result.patternOptions->markerSize    = metaData.get<int>("markerSize", 0);
            result.patternOptions->markerSpacing = metaData.get<int>("markerSpacing", 0);
        }
        result.reprojectionError = metaData.get<int>("reprojectionError", 0);
        return result;
    }

    void writeToFile(const std::filesystem::path& filePath) const {
        boost::property_tree::ptree ptree;
        if (cameraName.has_value()) {
            ptree.put("name", cameraName.value());
        }

        // Add resolution
        std::vector<int> resolution = {imageResolution.width, imageResolution.height};
        addListToTree(ptree, "resolution", resolution);

        // Add principal point
        std::vector<double> principalPt = {principalPoint.x, principalPoint.y};
        addListToTree(ptree, "principalPoint", principalPt);

        // Add focal length
        std::vector<double> fLength = {focalLength.x, focalLength.y};
        addListToTree(ptree, "focalLength", fLength);

        // Add distortion coefficients
        addListToTree(ptree, "distortionCoefficients", distortionCoefficients);
        ptree.put("distortionModel",
                  (distortionModel == DistortionModel::EQUIDISTANT ? "Equidistant"
                                                                   : "Radial tangential"));

        // Add subtree with meta data
        boost::property_tree::ptree metaData;

        if (patternOptions.has_value()) {
            if (patternOptions->patternType == kalibr::frontend::PatternType::APRIL_GRID) {
                metaData.put("patternType", "AprilGrid");
            } else if (patternOptions->patternType ==
                       kalibr::frontend::PatternType::ASYMMETRIC_CIRCLES_GRID) {
                metaData.put("patternType", "Asymmetric circles grid");
            } else {
                metaData.put("patternType", "Chessboard");
            }

            // Add pattern shape
            std::vector<int> patternShape = {patternOptions->patternSize.width,
                                             patternOptions->patternSize.height};
            addListToTree(metaData, "patternShape", patternShape);

            // Add pattern size/spacing
            metaData.put("markerSize", patternOptions->markerSize);
            metaData.put("markerSpacing", patternOptions->markerSpacing);
        }

        // Add calibration error
        metaData.put("reprojectionError", reprojectionError);
        ptree.add_child("metaData", metaData);

        boost::property_tree::write_json(filePath, ptree);
    }

    cv::Size imageResolution;
    cv::Point2d focalLength;
    cv::Point2d principalPoint;
    std::vector<double> distortionCoefficients;
    DistortionModel distortionModel;
    double reprojectionError;
    std::optional<kalibr::frontend::PatternOptions> patternOptions{std::nullopt};
    std::optional<std::string> cameraName{std::nullopt};

  private:
    template <typename BaseType>
    static void addListToTree(boost::property_tree::ptree& root,
                              const std::string& name,
                              const std::vector<BaseType>& list) {
        boost::property_tree::ptree listNode;
        for (const BaseType entry : list) {
            boost::property_tree::ptree listEntry;
            listEntry.put("", entry);
            listNode.push_back(std::make_pair("", listEntry));
        }
        root.add_child(name, listNode);
    }

    template <typename BaseType>
    static std::vector<BaseType> getListFromTree(boost::property_tree::ptree& root,
                                                 const std::string& name) {
        std::vector<BaseType> list;
        for (auto& val : root.get_child(name)) {
            list.push_back(val.second.get_value<BaseType>());
        }
        return list;
    }
};
}  // namespace kalibr::backend
