#include "navigation/generate_images.h"

// BoB robotics includes
#include "imgproc/dct_hash.h"

// Google Test
#include <gtest/gtest.h>

// Standard C++ includes
#include <numeric>

using namespace BoBRobotics::ImgProc::DCTHash;

TEST(DCT, computeHash)
{
    cv::Mat scratch;
    const auto dct = [&scratch](const cv::Mat &image) {
        image.convertTo(scratch, CV_32FC1, 1.0 / 255);
        return computeHash(scratch).to_ullong();
    };

    EXPECT_EQ(dct(TestImages[0]), 15170113775845029951ULL);
    EXPECT_EQ(dct(TestImages[1]), 17858815827266642603ULL);
    EXPECT_EQ(dct(TestImages[2]), 1975389874025492227ULL);
}

TEST(DCT, distance)
{
    EXPECT_EQ(distance(0, 0), 0);
    EXPECT_EQ(distance(0, 1), 1);
    EXPECT_EQ(distance(std::numeric_limits<uint64_t>::max(), 0), 64);
}
