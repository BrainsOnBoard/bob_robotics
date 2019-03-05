// Google Test
#include "gtest/gtest.h"

// Put this using declaration here so we don't have to bother putting it in the test headers
namespace BoBRobotics {}
using namespace BoBRobotics;

// Automatically generated includes for the files in test_headers
#include ".include_test_headers.h"

int
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
