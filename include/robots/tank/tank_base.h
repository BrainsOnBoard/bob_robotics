#pragma once

// BoB robotics includes
#include "common/circstat.h"
#include "common/macros.h"
#include "hid/joystick.h"

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

    void stopMoving()
    {
        tank(0.f, 0.f);
    }

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler(
                [this, deadZone](auto &joystick, HID::JAxis axis, float value) {
                    return this->onJoystickEvent(joystick, axis, value, deadZone);
                });
    }

    void drive(const HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
               float deadZone = 0.25f)
    {
        drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
              joystick.getState(HID::JAxis::LeftStickVertical),
              deadZone);
    }

    void controlWithThumbsticks(HID::JoystickBase<HID::JAxis, HID::JButton> &joystick)
    {
        joystick.addHandler(
                [this](auto &joystick, HID::JAxis axis, float) {
                    constexpr auto Left = HID::JAxis::LeftStickVertical;
                    constexpr auto Right = HID::JAxis::RightStickVertical;
                    if (axis == Left || axis == Right) {
                        this->tank(joystick.getState(Left), joystick.getState(Right));
                        return true;
                    }

                    return false;
                });
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

    auto getMaximumSpeed() const
    {
        return this->getMaximumSpeedProportion() * getAbsoluteMaximumSpeed();
    }

    radians_per_second_t getMaximumTurnSpeed() const
    {
        return this->getMaximumSpeedProportion() * this->getAbsoluteMaximumTurnSpeed();
    }

    radians_per_second_t getAbsoluteMaximumTurnSpeed() const
    {
        // max turn speed = v_max / r
        return radians_per_second_t{ (getAbsoluteMaximumSpeed() * 2 / getRobotWidth()).value() };
    }

    void setMaximumSpeedProportion(float value)
    {
        BOB_ASSERT(value >= -1.f && value <= 1.f);
        m_MaximumSpeedProportion = value;
    }

    float getMaximumSpeedProportion() const
    {
        return m_MaximumSpeedProportion;
    }

private:
    float m_MaximumSpeedProportion = 1.f;

    void drive(float x, float y, float deadZone)
    {
        const float halfPi = pi<float>() / 2.0f;

        const bool deadX = (fabs(x) < deadZone);
        const bool deadY = (fabs(y) < deadZone);
        if (deadX && deadY) {
            tank(0.0f, 0.0f);
        } else if (deadX) {
            tank(-y, -y);
        } else if (deadY) {
            tank(x, -x);
        } else {
            // If length of joystick vector places it in deadZone, stop motors
            float r = hypot(x, y);

            // By removing deadzone, we're preventing it being possible to drive at low speed
            // So subtract deadzone, rescale the result and clamp so it's back on (0,1)
            r = std::min(1.f, (r - deadZone) / (1.0f - deadZone));

            const float theta = atan2(x, -y);
            const float twoTheta = 2.0f * theta;

            // Drive motor
            if (theta >= 0.0f && theta < halfPi) {
                tank(r, r * cos(twoTheta));
            } else if (theta >= halfPi && theta < pi<float>()) {
                tank(-r * cos(twoTheta), -r);
            } else if (theta < 0.0f && theta >= -halfPi) {
                tank(r * cos(twoTheta), r);
            } else if (theta < -halfPi && theta >= -pi<float>()) {
                tank(-r, -r * cos(twoTheta));
            }
        }
    }

    bool onJoystickEvent(HID::JoystickBase<HID::JAxis, HID::JButton> &joystick,
                         HID::JAxis axis, float, float deadZone)
    {
        if (axis == HID::JAxis::LeftStickVertical || axis == HID::JAxis::LeftStickVertical) {
            // drive robot with joystick
            drive(joystick, deadZone);
            return true;
        }

        return false;
    }

    void tank(float left, float right)
    {
        static_cast<Derived *>(this)->tank(left, right);
    }

    meter_t getRobotWidth() const
    {
        return static_cast<const Derived *>(this)->getRobotWidth();
    }

    meters_per_second_t getAbsoluteMaximumSpeed() const
    {
        return static_cast<const Derived *>(this)->getAbsoluteMaximumSpeed();
    }

}; // TankBase
} // Tank
} // Robots
} // BoBRobotics
