#pragma once

// Google Test
#include "gtest/gtest.h"

// Little helper macro for floating-point comparisons with the units library
#define BOB_EXPECT_UNIT_T_EQ(val1, val2) \
    EXPECT_DOUBLE_EQ(val1.value(), static_cast<decltype(val1)>(val2).value())
