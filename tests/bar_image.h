#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

cv::Mat
createBarImage();

cv::Mat
getBarImageFloat();

static const cv::Mat_<uchar> BarImage = createBarImage();
static const cv::Mat_<float> BarImageFloat = getBarImageFloat();
