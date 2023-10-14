#pragma once

// BoB robotics includes
#include "imgproc/mask.h"
#include "video/randominput.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>


constexpr size_t NumTestImages = 100;
static const cv::Size TestImageSize{ 90, 10 };

static inline std::array<cv::Mat, NumTestImages> generateImages()
{
    BoBRobotics::Video::RandomInput<> video{ TestImageSize, /*seed=*/42 };
    std::array<cv::Mat, NumTestImages> images;
    for (auto &image : images) {
        video.readGreyscaleFrameSync(image);
    }

    return images;
}
static const auto TestImages = generateImages();

static inline auto generateMask()
{
    cv::Mat mask{ TestImageSize, CV_8UC1, cv::Scalar(255.0) };
    cv::Mat subset{ mask, cv::Rect{ 0, 0, 10, 5 } };
    subset = cv::Scalar{ 0.0 };
    return BoBRobotics::ImgProc::Mask{ std::move(mask) };
}
static const auto TestMask = generateMask();
