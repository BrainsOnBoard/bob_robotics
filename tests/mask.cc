#include "common.h"

// BoB robotics includes
#include "imgproc/mask.h"

// Standard C++ includes
#include <algorithm>

using namespace BoBRobotics;

class Mask
  : public ::testing::Test
{
protected:
    const cv::Size m_Size{ 8, 4 };
    ImgProc::Mask m_EmptyMask;
    ImgProc::Mask m_ExampleMask;

    void SetUp() override
    {
        cv::Mat ex{ m_Size, CV_8UC1, cv::Scalar(0xff) };
        ex.colRange(0, m_Size.width / 2) = cv::Scalar(0x00);
    }
};

bool
equals(const cv::Mat &m1, const cv::Mat &m2)
{
    if (m1.type() != m2.type()) {
        return false;
    }
    if (m1.size() != m2.size()) {
        return false;
    }

    // Byte-wise comparison
    return std::equal(m1.datastart, m1.dataend, m2.datastart);
}

TEST_F(Mask, apply)
{
    cv::Mat matResult, matTest{ m_Size, CV_8UC1, cv::Scalar(0x00) };
    matTest.rowRange(0, m_Size.height / 2) = cv::Scalar(0xff);

    // Applying an empty mask should have no effect
    m_EmptyMask.apply(matTest, matResult);
    EXPECT_TRUE(equals(matResult, matTest));

    cv::Mat matExpected = matTest; // NOLINT
    matExpected.colRange(0, m_Size.width / 2) = cv::Scalar(0x00);

    m_ExampleMask.apply(matTest, matResult);
    EXPECT_TRUE(equals(matResult, matExpected));
}

TEST_F(Mask, combine)
{
    {
        ImgProc::Mask combined1;
        m_ExampleMask.combine(m_EmptyMask, combined1);
        EXPECT_TRUE(equals(m_ExampleMask.get(), combined1.get()));
    }

    {
        ImgProc::Mask combined2;
        m_EmptyMask.combine(m_ExampleMask, combined2);
        EXPECT_TRUE(equals(m_ExampleMask.get(), combined2.get()));
    }

    {
        cv::Mat otherMask{ m_Size, CV_8UC1, cv::Scalar(0x01) };
        otherMask.rowRange(0, m_Size.height / 2) = cv::Scalar(0x00);
        cv::Mat expected = otherMask; // NOLINT
        expected.colRange(0, m_Size.width / 2) = cv::Scalar(0x00);

        ImgProc::Mask combined3;
        m_ExampleMask.combine(ImgProc::Mask{ otherMask }, combined3);
        EXPECT_TRUE(equals(combined3.get(), expected));
    }
}
