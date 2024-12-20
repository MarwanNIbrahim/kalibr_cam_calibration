#include <CLI/CLI.hpp>
#include <opencv2/imgproc.hpp>
#include "utilities/kalibr_backend/kalibr_calibrator.hpp"
#include "utilities/kalibr_backend/ocv_calibrator.hpp"
#include "utilities/kalibr_frontend/marker_detector.hpp"

struct CLIArgs {
    kalibr::frontend::PatternOptions patternOptions;
    kalibr::backend::DistortionModel distortionModel;
    size_t numCalibImages;
    std::filesystem::path pathToImages;
    bool enablePreview;
};

CLIArgs parseArgs(int ac, char** av) {
    char patType, distModel;
    CLIArgs args;

    CLI::App app{"Run camera calibration given a sequence of images"};
    app.add_option("--patternType",
                   patType,
                   "Type of calibration pattern used. a for april grid, g for asymmetric circles "
                   "grid, or c for chessboard")
        ->default_val('a');
    app.add_option("--numRows",
                   args.patternOptions.patternSize.width,
                   "Number of rows for calibration pattern")
        ->default_val(6);
    app.add_option("--numCols",
                   args.patternOptions.patternSize.height,
                   "Number of rows for calibration pattern")
        ->default_val(6);
    app.add_option("--markerSize",
                   args.patternOptions.markerSize,
                   "Size of calibration markers used [in m]")
        ->default_val(0.03f);
    app.add_option("--markerSpacing",
                   args.patternOptions.markerSpacing,
                   "Spacing between used calibration markers [in m]. Only used if calibration "
                   "pattern is april grid")
        ->default_val(0.009f);
    app.add_option("-d, --distortionModel",
                   distModel,
                   "Type of lens distortion model used. Either r for radial tangential or e for "
                   "equidistant")
        ->default_val('e');
    app.add_option(
           "--numImages", args.numCalibImages, "Maximum number of images used for calibration")
        ->default_val(0);
    app.add_option("-p, --pathToImages", args.pathToImages, "Base path to png images")->required();
    app.add_option("--enablePreview", args.enablePreview, "Enable preview of detected images")
        ->default_val(true);

    if (patType == 'a') {
        args.patternOptions.patternType = kalibr::frontend::PatternType::APRIL_GRID;
    } else if (patType == 'c') {
        args.patternOptions.patternType = kalibr::frontend::PatternType::CHESSBOARD;
    } else if (patType = 'g') {
        args.patternOptions.patternType = kalibr::frontend::PatternType::ASYMMETRIC_CIRCLES_GRID;
    } else {
        throw std::invalid_argument(
            "Invalid pattern type provided. Should be either a for april grid or c for chessboard");
    }

    if (distModel == 'e') {
        args.distortionModel = kalibr::backend::DistortionModel::EQUIDISTANT;
    } else if (distModel == 'r') {
        args.distortionModel = kalibr::backend::DistortionModel::RADIAL_TANGENTIAL;
    } else {
        throw std::invalid_argument(
            "Invalid distortion model provided. Should be either e for equidistant or r for radial "
            "tangential");
    }

    app.parse(ac, av);

    return args;
}

int main(int ac, char** av) {
    const CLIArgs args = parseArgs(ac, av);

    std::cout << "Running calibration on images from: " << args.pathToImages << "\n";
    std::filesystem::path outputFilePath = args.pathToImages / "CalibResult";

    if (!std::filesystem::exists(outputFilePath)) {
        std::filesystem::create_directories(outputFilePath);
    }

    if (args.distortionModel == kalibr::backend::DistortionModel::EQUIDISTANT) {
        outputFilePath = outputFilePath / "calib_Eq.json";
    } else {
        outputFilePath = outputFilePath / "calib_RadTan.json";
    }

    // Pattern parameters
    cv::Size resolution;
    size_t numTotalImages = 0;

    for (const auto& entry : std::filesystem::directory_iterator(args.pathToImages)) {
        const auto imagePath = entry.path();
        if (imagePath.extension() != ".png") {
            continue;
        }
        if (numTotalImages == 0) {
            cv::Mat I  = cv::imread(imagePath);
            resolution = I.size();
        }
        ++numTotalImages;
    }

    if (numTotalImages == 0) {
        std::cout << "Found no png images in provided path\n";
        return EXIT_FAILURE;
    }

    // Read images using OpenCV
    kalibr::backend::CalibrationResult initialGuess;
    initialGuess.focalLength            = cv::Point2f(resolution.width, resolution.width);
    initialGuess.principalPoint         = cv::Point2f(resolution.width / 2, resolution.height / 2);
    initialGuess.distortionCoefficients = {0.0, 0.0, 0.0, 0.0};
    const kalibr::backend::CalibrationFlags flags = kalibr::backend::CalibrationFlags::NONE;

    kalibr::frontend::MarkerDetector detector(resolution, args.patternOptions);
    kalibr::backend::KalibrCalibrator calibrator(args.distortionModel, initialGuess, flags);
    for (size_t i = 0; i < numTotalImages; ++i) {
        const std::filesystem::path imagePath =
            args.pathToImages / ("Image_" + std::to_string(i) + ".png");
        cv::Mat I = cv::imread(imagePath);
        if (I.size().area() == 0) {
            continue;
        }

        kalibr::frontend::StampedImage image;
        if (I.channels() == 3) {
            cv::Mat gray;
            cv::cvtColor(I, gray, cv::COLOR_BGR2GRAY);
            image = kalibr::frontend::StampedImage(gray, i);
        } else {
            image = kalibr::frontend::StampedImage(I, i);
        }

        const auto observation = detector.detectPattern(image);
        calibrator.addObservation(observation);

        kalibr::frontend::MarkerDetector::DrawObservation(I, observation);

        if (calibrator.getNumValidObservations() >= args.numCalibImages &&
            args.numCalibImages > 0) {
            break;
        }

        if (!args.enablePreview) {
            continue;
        }

        // Output preview image
        cv::imshow("Image", I);
        const auto key = cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 'p') {
            cv::waitKey(0);
        }
    }

    auto result           = calibrator.calibrateCamera();
    result.cameraName     = "cam0";
    result.patternOptions = args.patternOptions;
    result.writeToFile(outputFilePath);

    return EXIT_SUCCESS;
}
