#pragma once

// BoB robotics includes
#include "video/randominput.h"

// Standard C++ includes
#include <array>

constexpr size_t NumTestImages = 100;
static const cv::Size TestImageSize{ 180, 720 };
static std::array<cv::Mat, NumTestImages> TestImages;

static inline void generateImages()
{
    BoBRobotics::Video::RandomInput<> video{ TestImageSize, "random", /*seed=*/42 };
    for (auto &image : TestImages) {
        video.readGreyscaleFrameSync(image);
    }
}
