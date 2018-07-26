#pragma once

// Third-party includes
#include "../third_party/units.h"

//----------------------------------------------------------------------------
// Macros
//----------------------------------------------------------------------------
#define BUFFER_OFFSET(i) ((void*)(i))

namespace BoBRobotics {
namespace AntWorld {

//! Convert a GLFloat to a meter_t
inline constexpr units::length::meter_t makeM(const GLfloat value)
{
    return units::make_unit<units::length::meter_t>(value);
}

//! Get the distance between two 2D points
template<class T1, class T2>
auto distance(const T1 &v1, const T2 &v2)
{
    return hypot(v2[1] - v1[1], v2[0] - v1[0]);
}

//! Get the distance between two 2D points
template<class T>
units::length::meter_t distance(const T &v1, units::length::meter_t x2, units::length::meter_t y2)
{
    return units::math::hypot(y2 - makeM(v1[1]), x2 - makeM(v1[0]));
}
}    // AntWorld
}    // BoBRobotics