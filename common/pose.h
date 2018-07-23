#pragma once

// Third-party includes
#include "../third_party/units.h"

using namespace units::literals;
using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Pose {

// A generic vector template
template<class T>
using Vector3 = T[3];

/*
 * A class for objects which can return their position in space.
 */
template<class LengthUnit>
class HasPosition
{
public:
    virtual Vector3<LengthUnit> &getPosition() = 0;
}; // HasPosition

/* 
 * A class for objects which can return their attitude (rotation).
 */
template<class AngleUnit>
class HasAttitude
{
public:
    virtual Vector3<AngleUnit> &getAttitude() = 0;
};

/*
 * A class for objects whose position in space can be set.
 */
template<class LengthUnit>
class Positionable
  : public HasPosition<LengthUnit>
{
public:
    virtual void setPosition(LengthUnit x, LengthUnit y, LengthUnit z) = 0;
};

/*
 * A class for objects which can be rotated in space.
 */
template<class AngleUnit>
class Rotatable
  : public HasAttitude<AngleUnit>
{
public:
    virtual void setAttitude(AngleUnit yaw, AngleUnit pitch, AngleUnit roll) = 0;
};
} // Pose
} // BoBRobotics