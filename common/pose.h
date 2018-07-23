#pragma once

// Standard C++ includes
#include <array>

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Pose {

// A generic template for unit arrays
template<class T>
using Vector3 = std::array<T, 3>;

/*
 * Returns a triple of unit-type objects.
 */
template<class OutputUnit, class ArrayType>
inline constexpr Vector3<OutputUnit>
makeUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // Pose
} // BoBRobotics