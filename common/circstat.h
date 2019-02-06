#pragma once

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
using namespace units::literals;

template<class T = double>
constexpr T
pi()
{
    return T(3.1415926535897932385L);
}

template<typename AngleType>
auto
normaliseAngle180(AngleType angle)
{
    static_assert(units::traits::is_angle_unit<AngleType>::value,
                  "AngleType is not a unit of angle");

    while (angle < -180_deg) {
        angle += 360_deg;
    }
    while (angle > 180_deg) {
        angle -= 360_deg;
    }

    return angle;
}

template<typename AngleType>
auto
circularDistance(AngleType angle1, AngleType angle2)
{
    return normaliseAngle180(angle1 - angle2);
}
} // BoBRobotics