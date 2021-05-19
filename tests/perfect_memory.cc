#include "common.h"
#include "test_algo.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"
#include "navigation/perfect_memory_store_hog.h"

using namespace BoBRobotics::Navigation;

TEST(PerfectMemory, SampleImage)
{
    testAlgo<PerfectMemoryRotater<>>("pm.bin");
}

TEST(PerfectMemory, SampleImageRMS)
{
    testAlgo<PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>>>("pm_rms.bin");
}

TEST(PerfectMemory, SampleImageHOG)
{
    /*
     * Using GTest's default EXPECT_FLOAT_EQ() method gives test failures when
     * this test is run through Jenkins, though not on my local machine,
     * presumably due to differences in how floating-point numbers are
     * processed. So just choose an explicit (conservative) level of precision
     * for this test.
     *      -- AD
     */
    constexpr float precision = 1e-7f;

    using namespace BoBRobotics;

    const auto filepath = Path::getProgramDirectory() / "navigation" / "pm_hog.bin";
    const auto trueDifferences = readMatrix<float>(filepath);

    PerfectMemoryRotater<PerfectMemoryStore::HOG<>> algo{ TestImageSize, cv::Size(10, 10), 8 };
    for (const auto &image : TestImages) {
        algo.train(image);
    }

    const auto &differences = algo.getImageDifferences(TestImages[0]);
    BOB_ASSERT(trueDifferences.size() == differences.size());
    for (int snap = 0; snap < differences.rows(); snap++) {
        for (int col = 0; col < differences.cols(); col++) {
            EXPECT_NEAR(differences(snap, col), trueDifferences(snap, col), precision);
        }
    }
}
