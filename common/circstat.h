#pragma once

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
using namespace units::angle;

template<typename AngleUnit>
radian_t
circularDistance(AngleUnit ang1, AngleUnit ang2)
{
    return units::math::atan2(units::math::sin(ang1 - ang2), units::math::cos(ang1 - ang2));
}

template<typename T1, typename T2>
radian_t
circularMean(const T1 &angles, const T2 &weights)
{
    double sumCos = 0.0;
    double sumSin = 0.0;
    for (size_t i = 0; i < angles.size(); i++) {
        sumCos += weights[i] * units::math::cos(angles[i]);
        sumSin += weights[i] * units::math::sin(angles[i]);
    }

    return units::math::atan2(sumSin / angles.size(), sumCos / angles.size());
}
} // BoBRobotics