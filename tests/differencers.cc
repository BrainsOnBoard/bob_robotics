#include "common.h"

// BoB robotics includes
#include "navigation/differencers.h"

// Standard C++ includes
#include <algorithm>

using namespace BoBRobotics::Navigation;

template<class Differencer>
void
compare(const cv::Mat &m1, const cv::Mat &m2, const cv::Mat &expected,
        float expectedMean, const cv::Mat &mask1 = {}, const cv::Mat &mask2 = {})
{
    ASSERT_EQ(m1.type(), CV_8UC1);
    ASSERT_EQ(m2.type(), CV_8UC1);
    ASSERT_EQ(expected.type(), CV_8UC1);
    ASSERT_EQ(m1.size(), m2.size());

    cv::Mat scratch;
    typename Differencer::template Internal<> differencer;
    size_t count = differencer.calculate(m1, m2, scratch, mask1, mask2);
    EXPECT_TRUE(std::equal(expected.datastart, expected.dataend, scratch.datastart));

    EXPECT_FLOAT_EQ(differencer.mean(scratch, count, mask1, mask2), expectedMean);
}

const cv::Size size{ 8, 4 };
const auto zeros = cv::Mat::zeros(size, CV_8UC1);
const auto ones = cv::Mat::ones(size, CV_8UC1);

TEST(Differencers, AbsDiff)
{
    compare<AbsDiff>(zeros, zeros, zeros, 0.f);
    compare<AbsDiff>(ones, ones, zeros, 0.f);
    compare<AbsDiff>(zeros, ones, ones, 1.f);
}

TEST(Differencers, AbsDiffMask)
{
    cv::Mat half = zeros;
    half.rowRange(0, half.rows / 2) = 1;

    cv::Mat mask{ size, CV_8UC1, 255 };
    mask.colRange(0, mask.cols / 2) = 0;
    compare<AbsDiff>(zeros, zeros, zeros, 0.f, mask, mask);
    compare<AbsDiff>(ones, ones, zeros, 0.f, mask, mask);
    compare<AbsDiff>(zeros, ones, ones, 1.f, mask, mask);
    compare<AbsDiff>(zeros, half, half, 0.5f, mask, mask);

    cv::Mat mask2{ size, CV_8UC1, 255 };
    mask2.row(0) = 0;
    compare<AbsDiff>(zeros, half, half, 1.f / 3.f, mask2, mask2);
}

TEST(Differencers, RMSDiff)
{
    compare<RMSDiff>(zeros, zeros, zeros, 0.f);
    compare<RMSDiff>(ones, ones, zeros, 0.f);
    compare<RMSDiff>(zeros, ones, ones, 1.f);
}

TEST(Differencers, RMSDiffMask)
{
    cv::Mat half = zeros;
    half.rowRange(0, half.rows / 2) = 1;

    cv::Mat mask{ size, CV_8UC1, 255 };
    mask.colRange(0, mask.cols / 2) = 0;
    compare<AbsDiff>(zeros, zeros, zeros, 0.f, mask, mask);
    compare<AbsDiff>(ones, ones, zeros, 0.f, mask, mask);
    compare<AbsDiff>(zeros, ones, ones, 1.f, mask, mask);
    compare<AbsDiff>(zeros, half, half, 0.5f, mask, mask);

    cv::Mat mask2{ size, CV_8UC1, 255 };
    mask2.row(0) = 0;
    compare<AbsDiff>(zeros, half, half, 1.f / 3.f, mask2, mask2);
}
