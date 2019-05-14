#pragma once

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
using namespace units::literals;

template<class T = double>
constexpr T
pi()
{
    return T(3.1415926535897932385L);
}

template<typename AngleType>
constexpr auto
normaliseAngle180(AngleType angle)
{
    static_assert(units::traits::is_angle_unit<AngleType>::value,
                  "AngleType is not a unit of angle");

    while (angle <= -180_deg) {
        angle += 360_deg;
    }
    while (angle > 180_deg) {
        angle -= 360_deg;
    }

    return angle;
}

template<typename AngleType>
constexpr auto
normaliseAngle360(AngleType angle)
{
    static_assert(units::traits::is_angle_unit<AngleType>::value,
                  "AngleType is not a unit of angle");

    while (angle < 0_deg) {
        angle += 360_deg;
    }
    while (angle > 360_deg) {
        angle -= 360_deg;
    }

    return angle;
}

template<typename T>
constexpr auto
circularMean(const T &angles)
{
    units::dimensionless::scalar_t sumCos = 0.0, sumSin = 0.0;
    for (size_t i = 0; i < angles.size(); i++) {
        sumCos += units::math::cos(angles[i]);
        sumSin += units::math::sin(angles[i]);
    }

    return units::math::atan2(sumSin / angles.size(), sumCos / angles.size());
}

template<typename T1, typename T2>
constexpr auto
circularMean(const T1 &angles, const T2 &weights)
{
    units::dimensionless::scalar_t sumCos = 0.0, sumSin = 0.0;
    for (size_t i = 0; i < angles.size(); i++) {
        sumCos += weights[i] * units::math::cos(angles[i]);
        sumSin += weights[i] * units::math::sin(angles[i]);
    }

    return units::math::atan2(sumSin / angles.size(), sumCos / angles.size());
}

template<typename AngleType1, typename AngleType2>
constexpr auto
circularDistance(AngleType1 angle1, AngleType2 angle2)
{
    return normaliseAngle180(angle1 - angle2);
}

} // BoBRobotics