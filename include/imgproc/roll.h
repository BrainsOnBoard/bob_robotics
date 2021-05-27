#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstddef>

namespace BoBRobotics {
namespace ImgProc {
//! Roll a panoramic image leftwards by the specified number of pixels
void
roll(const cv::Mat &in, cv::Mat &out, size_t pixelsLeft);
} // ImgProc
} // BoBRobotics
