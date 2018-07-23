#pragma once

// Standard C++ includes
#include <tuple>

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Pose {

// A generic vector template
template<class T>
using Triple = std::tuple<T, T, T>;

// A generic template for unit arrays
template<class T>
using Array3 = T[3];

/*
 * Returns a triple of unit-type objects.
 */
template<class OutputUnit, class ArrayType>
inline constexpr Triple<OutputUnit>
makeUnitTriple(const ArrayType &values)
{
    return std::make_tuple(static_cast<OutputUnit>(values[0]),
                           static_cast<OutputUnit>(values[1]),
                           static_cast<OutputUnit>(values[2]));
}

// A little macro for defining our helper functions
#define DEFINE_GET_FUNCTION(NAME, NUMBER)        \
    template<class T>                            \
    inline constexpr auto &NAME(const T &triple) \
    {                                            \
        return std::get<NUMBER>(triple);         \
    }

// Helper functions for position triples
DEFINE_GET_FUNCTION(getX, 0)
DEFINE_GET_FUNCTION(getY, 1)
DEFINE_GET_FUNCTION(getZ, 2)

// Helper functions for rotation triples
DEFINE_GET_FUNCTION(getYaw, 0)
DEFINE_GET_FUNCTION(getPitch, 1)
DEFINE_GET_FUNCTION(getRoll, 2)

// Undefine macro now we're done with it
#undef DEFINE_GET_FUNCTION
} // Pose
} // BoBRobotics