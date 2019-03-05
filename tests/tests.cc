// Third-party includes
#include "third_party/units.h"

// Google Test
#include "gtest/gtest.h"

// Put these using declarations here so we don't have to bother putting it in the test headers
namespace BoBRobotics {}
using namespace BoBRobotics;
using namespace units::literals;

// Little helper macro for floating-point comparisons with the units library
#define ASSERT_UNIT_EQ(val1, val2) \
    ASSERT_DOUBLE_EQ(val1.value(), static_cast<decltype(val1)>(val2).value())

// Automatically generated includes for the files in test_headers
#include ".include_test_headers.h"

int
main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
