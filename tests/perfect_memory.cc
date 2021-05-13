#include "common.h"
#include "test_algo.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

using namespace BoBRobotics::Navigation;

TEST(PerfectMemory, SampleImage)
{
    testAlgo<PerfectMemoryRotater<>>("pm.bin");
}

TEST(PerfectMemory, SampleImageRMS)
{
    testAlgo<PerfectMemoryRotater<PerfectMemoryStore::RawImage<RMSDiff>>>("pm_rms.bin");
}
