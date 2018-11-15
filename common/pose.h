#pragma once

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <array>
#include <ostream>

namespace BoBRobotics {
//! A generic template for 2D unit arrays
template<typename T>
using Vector2 = std::array<T, 2>;

//! A generic template for 3D unit arrays
template<typename T>
using Vector3 = std::array<T, 3>;

//! A two-dimensional pose
template<typename LengthUnit, typename AngleUnit>
struct Pose2 {
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    static_assert(units::traits::is_angle_unit<AngleUnit>::value,
                  "AngleUnit is not a unit of angle");

    LengthUnit x{}, y{};
    AngleUnit angle{};

    Pose2() {}

    Pose2(LengthUnit xval, LengthUnit yval, AngleUnit angleval)
      : x(xval), y(yval), angle(angleval)
    {}
};

//! Converts the input array to a unit-type of OutputUnit
template<typename OutputUnit, typename ArrayType>
inline constexpr Vector3<OutputUnit>
convertUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // BoBRobotics
