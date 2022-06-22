#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace ImgProc {
void
convertScale(const cv::Mat &in, cv::Mat &out, int targetType);
} // ImgProc
} // BoBRobotics
