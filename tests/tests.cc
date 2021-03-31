// Google Test
#include "gtest/gtest.h"

int bobMain(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
