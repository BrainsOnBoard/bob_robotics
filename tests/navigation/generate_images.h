#pragma once

// BoB robotics includes
#include "video/randominput.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>


constexpr size_t NumTestImages = 100;
static const cv::Size TestImageSize{ 90, 10 };

static inline std::array<cv::Mat, NumTestImages> generateImages()
{
    BoBRobotics::Video::RandomInput<> video{ TestImageSize, "random", /*seed=*/42 };
    std::array<cv::Mat, NumTestImages> images;
    for (auto &image : images) {
        video.readGreyscaleFrameSync(image);
    }

    return images;
}

static const auto TestImages = generateImages();
