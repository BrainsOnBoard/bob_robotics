#include "common.h"
#include "test_algo.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_hog.h"

using namespace BoBRobotics::Navigation;
using Window = std::pair<size_t, size_t>;

#define PM_TEST_WINDOW(TEST_NAME, ALGO, FILENAME, MASK)      \
    TEST(PerfectMemory, TEST_NAME)                           \
    {                                                        \
        testAlgo<ALGO>(FILENAME, MASK, {});                  \
    }                                                        \
    TEST(PerfectMemory, TEST_NAME##Window)                   \
    {                                                        \
        testAlgo<ALGO>("window_" FILENAME, MASK, { 0, 10 }); \
    }

#define PM_TEST(TEST_NAME, ALGO, FILENAME)        \
    PM_TEST_WINDOW(TEST_NAME, ALGO, FILENAME, {}) \
    PM_TEST_WINDOW(TEST_NAME##Mask, ALGO, "mask_" FILENAME, TestMask)

PM_TEST(SampleImage, PerfectMemoryRotater<>, "pm.bin")
PM_TEST(SampleImageRMS, PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>>, "pm_rms.bin")

void testCCoeff(const std::string &filename, const ImgProc::Mask &mask, std::pair<size_t, size_t> window)
{
    using namespace BoBRobotics;
    constexpr float precision = 1e-5f;

    const auto filepath = getTestsPath() / "navigation" / filename;
    const auto trueDifferences = readMatrix<float>(filepath);

    PerfectMemoryRotater<PerfectMemoryStore::RawImage<CorrCoefficient>> algo{ TestImageSize };
    for (const auto &image : TestImages) {
        algo.train(image, mask);
    }

    if (window == Window{}) {
        window = algo.getFullWindow();
    }
    const auto &differences = algo.getImageDifferences(TestImages[0], mask, window);
    compareFloatMatrices(differences, trueDifferences, precision);
}

TEST(PerfectMemory, SampleImageCCoeff)
{
    testCCoeff("pm_ccoeff.bin", {}, {});
}

TEST(PerfectMemory, SampleImageCCoeffWindow)
{
    testCCoeff("window_pm_ccoeff.bin", {}, { 0, 10 });
}

/*
 * There seems to be a bug meaning that non-empty image masks break
 * cv::matchTemplate() when the cv::TM_CCOEFF_NORMED method is used for versions
 * of OpenCV < 4.5.2. Just disable the tests in this case.
 */
#ifdef BOB_OPENCV_SUPPORTS_CCOEFF_MASKS
TEST(PerfectMemory, SampleImageCCoeffMask)
{
    testCCoeff("mask_pm_ccoeff.bin", TestMask, {});
}

TEST(PerfectMemory, SampleImageCCoeffWindowMask)
{
    testCCoeff("window_mask_pm_ccoeff.bin", TestMask, { 0, 10 });
}
#else
TEST(PerfectMemory, DISABLED_SampleImageCCoeffMask)
{}

TEST(PerfectMemory, DISABLED_SampleImageCCoeffMaskWindow)
{}
#endif

template<class Store>
void
testHog(const std::string &filename, std::pair<size_t, size_t> window, float precision)
{
    /*
     * Using GTest's default EXPECT_FLOAT_EQ() method gives test failures when
     * this test is run through Jenkins, though not on my local machine,
     * presumably due to differences in how floating-point numbers are
     * processed. So just choose an explicit (conservative) level of precision
     * for this test.
     *      -- AD
     */

    using namespace BoBRobotics;

    const auto filepath = getTestsPath() / "navigation" / filename;
    const auto trueDifferences = readMatrix<float>(filepath);

    PerfectMemoryRotater<Store> algo{ TestImageSize, cv::Size(10, 10), 8 };
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    if (window == Window{}) {
        window = algo.getFullWindow();
    }
    const auto &differences = algo.getImageDifferences(TestImages[0], ImgProc::Mask{}, window);
    compareFloatMatrices(differences, trueDifferences, precision);
}

TEST(PerfectMemory, SampleImageHOG)
{
    testHog<PerfectMemoryStore::HOG<>>("pm_hog.bin", {}, 1e-7);
}

TEST(PerfectMemory, SampleImageHogWindow)
{
    testHog<PerfectMemoryStore::HOG<>>("window_pm_hog.bin", { 0, 10 }, 1e-7);
}

TEST(PerfectMemory, SampleImageHOGCorrCoefficient)
{
    testHog<PerfectMemoryStore::HOG<CorrCoefficient>>("pm_hog_ccoeff.bin", {}, 1e-5);
}

TEST(PerfectMemory, SampleImageHogCorrCoefficientWindow)
{
    testHog<PerfectMemoryStore::HOG<CorrCoefficient>>("window_pm_hog_ccoeff.bin", { 0, 10 }, 1e-5);
}
