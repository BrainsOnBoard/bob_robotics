#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

cv::Mat
createBarImage();

static const cv::Mat_<uchar> BarImage = createBarImage();
