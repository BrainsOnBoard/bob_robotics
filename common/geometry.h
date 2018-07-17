#pragma once

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::math;
using namespace units::angle;
using namespace units::length;

#define makeM(x) units::make_unit<meter_t>(x)

namespace BoBRobotics {
namespace Geometry {
class Vector2m
{
private:
    std::array<meter_t, 2> values;

public:
    meter_t &X;
    meter_t &Y;
    static constexpr meter_t Z = 0_m;

    Vector2m(std::array<meter_t, 2> values)
      : values(values)
      , X(values[0])
      , Y(values[1])
    {}

    Vector2m(meter_t x, meter_t y)
      : values{ x, y }
      , X(values[0])
      , Y(values[1])
    {}

    Vector2m()
      : Vector2m(0_m, 0_m)
    {}

    meter_t &operator[](size_t i)
    {
        return values[i];
    }

    meter_t operator[](size_t i) const
    {
        return values[i];
    }

    Vector2m &operator=(const Vector2m &vector)
    {
        X = vector.X;
        Y = vector.Y;
        return *this;
    }
};

class Vector3m
{
private:
    std::array<meter_t, 3> values;

public:
    meter_t &X;
    meter_t &Y;
    meter_t &Z;

    Vector3m(std::array<meter_t, 3> values)
      : values(values)
      , X(values[0])
      , Y(values[1])
      , Z(values[2])
    {}

    Vector3m(meter_t x, meter_t y, meter_t z)
      : values{ x, y, z }
      , X(values[0])
      , Y(values[1])
      , Z(values[2])
    {}

    Vector3m()
      : Vector3m(0_m, 0_m, 0_m)
    {}

    meter_t &operator[](size_t i)
    {
        return values[i];
    }

    meter_t operator[](size_t i) const
    {
        return values[i];
    }
};

template<class T1, class T2>
auto
distance2(const T1 &v1, const T2 &v2)
{
    return hypot(v2[1] - v1[1], v2[0] - v1[0]);
}

template<class T>
meter_t
distance2(const T &v1, meter_t x2, meter_t y2)
{
    return hypot(y2 - makeM(v1[1]), x2 - makeM(v1[0]));
}
}
}