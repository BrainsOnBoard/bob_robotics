#include "common.h"
#include "test_algo.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

TEST(PerfectMemory, SampleImage)
{
    testAlgo<BoBRobotics::Navigation::PerfectMemoryRotater<>>("pm.bin");
}
