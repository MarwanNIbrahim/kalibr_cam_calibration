import kalibr
import argparse
from dataclasses import dataclass
import os
import cv2


@dataclass
class CLIArgs():
    patternOptions: kalibr.frontend.PatternOptions = None
    distortionModel: kalibr.backend.DistortionModel = kalibr.backend.DistortionModel.EQUIDISTANT
    numCalibImages: int = 0
    pathToImages: str = ""
    enablePreview: bool = True


def parse_args() -> CLIArgs:
    parser = argparse.ArgumentParser(
        description="Script for running monocular camera calibration on a set of images")
    parser.add_argument(
        "--patternType",
        default='a',
        help=
        "Type of calibration pattern used. a for April grid, g for asymmetric circles grid, or c for chessboard"
    )
    parser.add_argument("--numRows", default=6, help="Number of rows in calibration pattern")
    parser.add_argument("--numCols", default=6, help="Number of columns in calibration pattern")
    parser.add_argument("--markerSize", default=0.03, help="Size of calibration marker [in m]")
    parser.add_argument("--markerSpacing",
                        default=0.009,
                        help="Spacing between calibration markers [in m]")
    parser.add_argument("-d",
                        "--distortionModel",
                        default='e',
                        help="Distortion model used. r for Radial tangential, or e for Equidistant")
    parser.add_argument(
        "--numImages",
        default=0,
        help="Maximum number of images used for calibration. Set to a number <=0 to disable")
    parser.add_argument("-p", "--pathToImages", help="Base path to png images")
    parser.add_argument("--enablePreview", default=True, help="Enable/disable image preview")

    args = parser.parse_args()
    structArgs = CLIArgs()
    structArgs.patternOptions = kalibr.frontend.PatternOptions()

    if args.patternType == 'a':
        structArgs.patternOptions.patternType = kalibr.frontend.PatternType.APRIL_GRID
    elif args.patternType == 'c':
        structArgs.patternOptions.patternType = kalibr.frontend.PatternType.CHESSBOARD
    elif args.patternType == 'g':
        structArgs.patternOptions.patternType = kalibr.frontend.PatternType.ASYMMETRIC_CIRCLES_GRID
    else:
        raise ValueError("Unsupported pattern type provided")

    if args.distortionModel == 'r':
        structArgs.distortionModel = kalibr.backend.DistortionModel.RADIAL_TANGENTIAL
    elif args.distortionModel == 'e':
        structArgs.distortionModel = kalibr.backend.DistortionModel.EQUIDISTANT
    else:
        raise ValueError("Unsupported distortion model provided")

    structArgs.patternOptions.patternSize = (args.numRows, args.numCols)
    structArgs.patternOptions.markerSize = args.markerSize
    structArgs.patternOptions.markerSpacing = args.markerSpacing
    structArgs.pathToImages = args.pathToImages
    structArgs.enablePreview = bool(args.enablePreview)
    return structArgs


def main():
    args = parse_args()

    print(f"Running calibration on images from {args.pathToImages}")

    outputFilePath = os.path.join(args.pathToImages, "CalibResult")
    if not os.path.exists(outputFilePath):
        os.makedirs(outputFilePath)

    if args.distortionModel == kalibr.backend.DistortionModel.EQUIDISTANT:
        outputFilePath = os.path.join(outputFilePath, "calib_Eq.json")
    else:
        outputFilePath = os.path.join(outputFilePath, "calib_RadTan.json")

    resolution = None
    numTotalImages = 0
    for f in os.listdir(args.pathToImages):
        if not f.endswith(".jpeg") and not f.endswith(".png"):
            continue

        imagePath = os.path.join(args.pathToImages, f)
        if numTotalImages == 0:
            I = cv2.imread(imagePath)
            resolution = I.shape[:2][::-1]
        numTotalImages = numTotalImages + 1

    #Read images using OpenCV
    detector = kalibr.frontend.MarkerDetector(resolution, args.patternOptions)
    calibrator = kalibr.backend.KalibrCalibrator(args.distortionModel, None,
                                                 kalibr.backend.CalibrationFlags.NONE)

    for i in range(numTotalImages):
        imagePath = os.path.join(args.pathToImages, f"Image_{i}.png")
        if not os.path.exists(imagePath):
            imagePath = os.path.join(args.pathToImages, f"Image_{i}.jpeg")
        I = cv2.imread(imagePath)
        if (I.shape[0] == 0):
            continue

        image = kalibr.frontend.StampedImage()
        image.timestamp = i
        if len(I.shape) > 2:
            image.image = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
        else:
            image.image = I.copy()

        observation = detector.detectPattern(image)
        calibrator.addObservation(observation)

        kalibr.frontend.MarkerDetector.DrawObservation(I, observation)

        if calibrator.getNumValidObservations() >= args.numCalibImages and args.numCalibImages > 0:
            break

        if not args.enablePreview:
            continue

        cv2.imshow("Image", I)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    result = calibrator.calibrateCamera()
    result.cameraName = "cam0"
    result.patternOptions = args.patternOptions
    result.writeToFile(outputFilePath)


if __name__ == "__main__":
    main()
