#pragma once

#include <opencv2/opencv.hpp>

namespace kalibr::frontend {

enum PatternType { CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, APRIL_GRID };

struct PatternOptions {
    PatternType patternType;
    cv::Size patternSize;
    double markerSize;
    double markerSpacing;
};

}  // namespace kalibr::frontend
