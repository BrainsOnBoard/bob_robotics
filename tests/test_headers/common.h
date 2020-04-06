#pragma once

// Third-party includes
#include "third_party/units.h"

// Google Test
#include "gtest/gtest.h"

// Standard C++ includes
#include <string>
#include <utility>

// Put these using declarations here so we don't have to bother putting it in the test headers
namespace BoBRobotics {}
using namespace BoBRobotics;
using namespace units::literals;
using namespace units::length;

// Little helper macro for floating-point comparisons with the units library
#define BOB_EXPECT_UNIT_T_EQ(val1, val2) \
    EXPECT_DOUBLE_EQ(val1.value(), static_cast<decltype(val1)>(val2).value())
