#pragma once

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank::TankBase
//----------------------------------------------------------------------------
//! Interface for driving wheeled robots with tank steering
template<class Derived>
class TankBase
{
/*
 * If these are declared private then they annoyingly conflict with "usings" in
 * derived classes.
 */
protected:
    using meter_t = units::length::meter_t;
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    void moveForward(float speed)
    {
        tank(speed, speed);
    }

    void turnOnTheSpot(float clockwiseSpeed)
    {
        tank(clockwiseSpeed, -clockwiseSpeed);
    }

    void move(float speed, float clockwiseSpeed)
    {
        tankMaxScaled(speed + clockwiseSpeed,
                      speed - clockwiseSpeed);
    }

    void stopMoving()
    {
        tank(0.f, 0.f);
    }

    void move(meters_per_second_t v,
              radians_per_second_t clockwiseSpeed,
              const bool maxScaled)
    {
        const meters_per_second_t diff{
            (clockwiseSpeed * getRobotWidth() / 2).value()
        };
        const meters_per_second_t vL = v + diff;
        const meters_per_second_t vR = v - diff;
        tank(vL, vR, maxScaled);
    }

    void tankMaxScaled(const float left, const float right, const float max = 1.f)
    {
        const float larger = std::max(std::fabs(left), std::fabs(right));
        if (larger <= max) {
            tank(left, right);
        } else {
            const float ratio = max / larger;
            tank(ratio * left, ratio * right);
        }
    }

    void tank(meters_per_second_t left, meters_per_second_t right, bool maxScaled = false)
    {
        const meters_per_second_t maxSpeed = getMaximumSpeed();
        const auto leftMotor = static_cast<float>(left / maxSpeed);
        const auto rightMotor = static_cast<float>(right / maxSpeed);
        if (maxScaled) {
            tankMaxScaled(leftMotor, rightMotor);
        } else {
            tank(leftMotor, rightMotor);
        }
    }

    void tank(float left, float right)
    {
        m_Left = left;
        m_Right = right;
        static_cast<Derived *>(this)->tankInternal(left, right);
    }

    radians_per_second_t getMaximumTurnSpeed() const
    {
        // max turn speed = v_max / r
        return radians_per_second_t{ (getMaximumSpeed() * 2 / getRobotWidth()).value() };
    }

    //! Get low-level left motor speed
    float getLeft() const{ return m_Left; }

    //! Get low-level right motor speed
    float getRight() const{ return m_Right; }

    //! Get forward movement speed
    float getForwardSpeed() const
    {
        return (getLeft() + getRight()) * 0.5f;
    }

    //! Get turning speed
    float getTurnSpeed() const
    {
        return (getLeft() - getRight()) * 0.5f;
    }

protected:
    TankBase()
    :   m_Left(0.0f), m_Right(0.0f)
    {}

private:
    meter_t getRobotWidth() const
    {
        return static_cast<const Derived *>(this)->getRobotWidth();
    }

    meters_per_second_t getMaximumSpeed() const
    {
        return static_cast<const Derived *>(this)->getMaximumSpeed();
    }

    float m_Left;
    float m_Right;

}; // TankBase
} // Tank
} // Robots
} // BoBRobotics
