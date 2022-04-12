#include "common.h"
#include "bar_image.h"

// Third-party includes
#include "range/v3/algorithm.hpp"
#include "range/v3/view.hpp"

// BoB robotics includes
#include "imgproc/roll.h"

using namespace ranges;

TEST(Roll, RollLeft)
{
    cv::Mat_<uchar> rolled;
    BoBRobotics::ImgProc::rollLeft(BarImage, rolled, 1);

    cv::Mat_<uchar> rolledExpected{ rolled.size(), CV_8U };
    rolledExpected = 255;
    rolledExpected.colRange(0, 2) = 0;

    EXPECT_TRUE(equal(views::all(rolled), views::all(rolledExpected)));
}

TEST(Roll, RollRight)
{
    cv::Mat_<uchar> rolled;
    BoBRobotics::ImgProc::rollRight(BarImage, rolled, 1);

    cv::Mat_<uchar> rolledExpected{ rolled.size(), CV_8U };
    rolledExpected = 255;
    rolledExpected.colRange(2, 4) = 0;

    EXPECT_TRUE(equal(views::all(rolled), views::all(rolledExpected)));
}
