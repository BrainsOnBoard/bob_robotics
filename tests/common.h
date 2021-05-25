#pragma once

// Google Test
#include "gtest/gtest.h"

// Little helper macro for floating-point comparisons with the units library
#define BOB_EXPECT_UNIT_T_EQ(val1, val2) \
    EXPECT_DOUBLE_EQ(val1.value(), static_cast<decltype(val1)>(val2).value())

template<class T, class U, class Pred>
void compareFloatMatricesPredicate(const T &m1, const U &m2, const Pred &compare)
{
    static_assert(std::is_same<const float &, const decltype(m1[0]) &>::value &&
                          std::is_same<const float &, const decltype(m2[0]) &>::value,
                  "m1 and m2 must contain floats");

    ASSERT_EQ(m1.size(), m2.size());

    SCOPED_TRACE("Comparing matrices");
    for (int i = 0; i < m1.rows(); i++) {
        for (int j = 0; j < m1.cols(); j++) {
            compare(m1(i, j), m2(i, j));
        }
    }
}

template<class T, class U>
void compareFloatMatrices(const T &m1, const U &m2)
{
    const auto expect = [](float a, float b) {
        EXPECT_FLOAT_EQ(a, b);
    };
    compareFloatMatricesPredicate(m1, m2, expect);
}

template<class T, class U>
void compareFloatMatrices(const T &m1, const U &m2, float precision)
{
    const auto near = [precision](float a, float b) {
        EXPECT_NEAR(a, b, precision);
    };
    compareFloatMatricesPredicate(m1, m2, near);
}
