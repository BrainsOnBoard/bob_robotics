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
    testAlgo<PerfectMemoryRotater<PerfectMemoryStore::HOG<>>>("pm_hog.bin", cv::Size(10, 10), 8);
}
