#pragma once

// Standard C++ includes
#include <array>

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
//! A generic template for 2D unit arrays
template<class T>
using Vector2 = std::array<T, 2>;

//! A generic template for 3D unit arrays
template<class T>
using Vector3 = std::array<T, 3>;

//! Converts the input array to a unit-type of OutputUnit
template<class OutputUnit, class ArrayType>
inline constexpr Vector3<OutputUnit>
convertUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // BoBRobotics