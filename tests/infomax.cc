#include "common.h"
#include "test_algo.h"
#include "navigation/infomax_test.h"

TEST(InfoMax, SampleImage)
{
    testAlgo<BoBRobotics::Navigation::InfoMaxTest>("infomax.bin");
}
