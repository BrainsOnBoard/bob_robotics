#include "common.h"

// BoB robotics includes
#include "imgproc/mask.h"
#include "navigation/differencers.h"

// Standard C++ includes
#include <algorithm>
#include <utility>

using namespace BoBRobotics;
using namespace BoBRobotics::Navigation;

#define COMPARE_MASK(m1, m2, expected, expectedMean, mask1, mask2)                    \
    do {                                                                              \
        ASSERT_EQ(m1.type(), CV_8UC1);                                                \
        ASSERT_EQ(m2.type(), CV_8UC1);                                                \
        ASSERT_EQ(expected.type(), CV_8UC1);                                          \
        ASSERT_EQ(m1.size(), m2.size());                                              \
        bool doesMatch;                                                               \
        float mean;                                                                   \
        std::tie(doesMatch, mean) = compare<AbsDiff>(m1, m2, expected, mask1, mask2); \
        EXPECT_TRUE(doesMatch);                                                       \
        EXPECT_TRUE(mean == expectedMean);                                            \
    } while (false);

#define COMPARE(m1, m2, expected, expectedMean) \
    COMPARE_MASK(m1, m2, expected, expectedMean, {}, {})

template<class Differencer>
std::pair<bool, float>
compare(const cv::Mat &m1, const cv::Mat &m2, const cv::Mat &expected,
        const ImgProc::Mask &mask1 = {},
        const ImgProc::Mask &mask2 = {})
{
    cv::Mat scratch;
    typename Differencer::template Internal<> differencer;
    size_t count = differencer.calculate(m1, m2, scratch, mask1, mask2);
    bool doesMatch = std::equal(expected.datastart, expected.dataend, scratch.datastart);
    float mean = differencer.mean(scratch, count, mask1, mask2);
    return { doesMatch, mean };
}

class Differencers
  : public ::testing::Test
{
protected:
    const cv::Size m_Size{ 8, 4 };
    const cv::Mat m_Zeros{ m_Size, CV_8UC1, cv::Scalar(0) };
    const cv::Mat m_Ones{ m_Size, CV_8UC1, cv::Scalar(1) };
    cv::Mat m_Half1{ m_Size, CV_8UC1, cv::Scalar(0) };
    cv::Mat m_Half2{ m_Size, CV_8UC1, cv::Scalar(0) };
    ImgProc::Mask m_Mask1, m_Mask2, m_ZeroMask, m_OneMask;

    void SetUp() override
    {
        m_Half1.rowRange(0, m_Half1.rows / 2) = 1;
        m_Half2.rowRange(m_Half2.rows / 2, m_Half2.rows) = 1;

        cv::Mat mask1{ m_Size, CV_8UC1, cv::Scalar(0xff) };
        mask1.colRange(0, mask1.cols / 2) = 0;
        m_Mask1.set(std::move(mask1));

        cv::Mat mask2{ m_Size, CV_8UC1, cv::Scalar(0xff) };
        mask2.row(0) = 0;
        m_Mask2.set(std::move(mask2));

        /*
         * NB: I originally implemented this by passing m_Zeros and m_Ones by
         * copy but for some reason OpenCV then zeroed all the original data
         * even though the members are marked const. Eww.
         */
        m_ZeroMask.set(cv::Mat_<uint8_t>::zeros(m_Size));
        m_OneMask.set(cv::Mat_<uint8_t>::ones(m_Size));
    }
};

TEST_F(Differencers, AbsDiff)
{
    COMPARE(m_Zeros, m_Zeros, m_Zeros, 0.f);
    COMPARE(m_Ones, m_Ones, m_Zeros, 0.f);
    const cv::Mat_<uint8_t> ones{ m_Size, 1 };
    COMPARE(m_Zeros, m_Ones, m_Ones, 1.f);
}

TEST_F(Differencers, AbsDiffMask)
{
    COMPARE_MASK(m_Zeros, m_Zeros, m_Zeros, 0.f, m_Mask1, m_Mask1);
    COMPARE_MASK(m_Ones, m_Ones, m_Zeros, 0.f, m_Mask1, m_Mask1);
    COMPARE_MASK(m_Zeros, m_Ones, m_Ones, 1.f, m_Mask1, m_Mask1);
    COMPARE_MASK(m_Zeros, m_Half1, m_Half1, 0.5f, m_Mask1, m_Mask1);
    COMPARE_MASK(m_Zeros, m_Half1, m_Half1, 1.f / 3.f, m_Mask2, m_Mask2);
}

TEST_F(Differencers, RMSDiff)
{
    RMSDiff::Internal<> rmsDiff;
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Zeros), 0.f);
    EXPECT_FLOAT_EQ(rmsDiff(m_Ones, m_Ones), 0.f);
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Ones), 1.f);

    // Verified with MATLAB
    const cv::Mat_<uint8_t> im1{ 208, 231, 32, 233, 161 };
    const cv::Mat_<uint8_t> im2{ 25, 71, 139, 244, 246 };
    EXPECT_FLOAT_EQ(rmsDiff(im1, im2), 124.8070510828615f);
}

TEST_F(Differencers, RMSDiffMask)
{
    RMSDiff::Internal<> rmsDiff;
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Zeros, m_Mask1), 0.f);
    EXPECT_FLOAT_EQ(rmsDiff(m_Ones, m_Ones, m_Mask1), 0.f);
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Ones, m_Mask1), 1.f);
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Half1, m_Mask1), sqrtf(0.5f));
    EXPECT_FLOAT_EQ(rmsDiff(m_Zeros, m_Half1, m_Mask2), sqrtf(1.f / 3.f));

    const cv::Mat_<uint8_t> im1{ 100, 208, 231, 32, 233, 161 };
    const cv::Mat_<uint8_t> im2{ 0, 25, 71, 139, 244, 246 };
    const ImgProc::Mask mask{ cv::Mat{ 0, 255, 255, 255, 255, 255 } };
    EXPECT_FLOAT_EQ(rmsDiff(im1, im2, mask), 124.8070510828615f);
}

TEST_F(Differencers, CorrCoefficient)
{
    CorrCoefficient::Internal<> ccoeff;

    /*
     * You can't calculate Pearson's rho if one of the input vectors is entirely
     * composed of the same value (you get a divide-by-zero).
     */
    EXPECT_THROW({ ccoeff(m_Zeros, m_Zeros); }, std::invalid_argument);
    EXPECT_THROW({ ccoeff(m_Zeros, m_Half1); }, std::invalid_argument);
    EXPECT_THROW({ ccoeff(m_Half1, m_Zeros); }, std::invalid_argument);

    /*
     * Note that we are using 1 - abs(Pearson's rho) to give us a difference
     * value.
     */
    EXPECT_FLOAT_EQ(ccoeff(m_Half1, m_Half1), 0.f);
    EXPECT_FLOAT_EQ(ccoeff(m_Half2, m_Half1), 0.f);

    // Verified with MATLAB
    const cv::Mat_<uint8_t> im1{ 167, 9, 217, 238, 173 };
    const cv::Mat_<uint8_t> im2{ 193, 189, 100, 167, 44 };
    EXPECT_FLOAT_EQ(ccoeff(im1, im2), 1.f - 0.362822774581930f);
}
