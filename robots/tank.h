#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../hid/joystick.h"
#include "../net/connection.h"
#include "robot.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <sstream>
#include <utility>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank
//----------------------------------------------------------------------------
//! Interface for driving wheeled robots with tank steering
class Tank
  : public Robot
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
    virtual void moveForward(float speed) override
    {
        tank(speed, speed);
    }

    virtual void turnOnTheSpot(float clockwiseSpeed) override
    {
        tank(clockwiseSpeed, -clockwiseSpeed);
    }

    virtual void stopMoving() override
    {
        tank(0.f, 0.f);
    }

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler(
                [this, deadZone](HID::JAxis axis, float value) {
                    return onJoystickEvent(axis, value, deadZone);
                });
    }

    virtual void controlWithThumbsticks(HID::Joystick &joystick)
    {
        joystick.addHandler(
                [this](HID::JAxis axis, float value) {
                    static float left{}, right{};

                    switch (axis) {
                    case HID::JAxis::LeftStickVertical:
                        left = -value;
                        break;
                    case HID::JAxis::RightStickVertical:
                        right = -value;
                        break;
                    default:
                        return false;
                    }

                    tank(left, right);
                    return true;
                });
    }

    void drive(const HID::Joystick &joystick, float deadZone = 0.25f)
    {
        drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
              joystick.getState(HID::JAxis::LeftStickVertical),
              deadZone);
    }

    void move(meters_per_second_t v,
              radians_per_second_t clockwiseSpeed)
    {
        const meter_t axisLength = getRobotWidth();
        const meters_per_second_t diff{
            (clockwiseSpeed * axisLength / 2).value()
        };
        const meters_per_second_t vL = v + diff;
        const meters_per_second_t vR = v - diff;
        tankVelocities(vL, vR);
    }

    //! Set the left and right motors to the specified speed
    virtual void tank(float left, float right)
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }

    void tankMaxScaled(const float left, const float right, const float max = 1.f)
    {
        const float larger = std::max(std::abs(left), std::abs(right));
        if (larger <= max) {
            tank(left, right);
        } else {
            const float ratio = max / larger;
            tank(ratio * left, ratio * right);
        }
    }

    virtual void tankVelocities(meters_per_second_t left, meters_per_second_t right)
    {
        const meters_per_second_t maxSpeed = getMaximumSpeed();
        const auto leftMotor = static_cast<float>(left / maxSpeed);
        const auto rightMotor = static_cast<float>(right / maxSpeed);
        tank(leftMotor, rightMotor);
    }

    virtual millimeter_t getRobotWidth()
    {
        throw std::runtime_error("getRobotWidth() is not implemented for this class");
    }

    virtual meters_per_second_t getMaximumSpeed()
    {
        throw std::runtime_error("getMaximumSpeed() is not implemented for this class");
    }

    virtual radians_per_second_t getMaximumTurnSpeed()
    {
        // max turn speed = v_max / r
        return radians_per_second_t{ (getMaximumSpeed() * 2 / static_cast<meter_t>(getRobotWidth())).value() };
    }

    //! Controls the robot with a network stream
    void readFromNetwork(Net::Connection &connection)
    {
        // Send robot parameters over network
        constexpr double _nan = std::numeric_limits<double>::quiet_NaN();
        double maxTurnSpeed = _nan, maxForwardSpeed = _nan, axisLength = _nan;
        try {
            maxTurnSpeed = getMaximumTurnSpeed().value();
        } catch (std::runtime_error &) {
            // Then getMaximumTurnSpeed() isn't implemented
        }
        try {
            maxForwardSpeed = getMaximumSpeed().value();
        } catch (std::runtime_error &) {
            // Then getMaximumSpeed() isn't implemented
        }
        try {
            axisLength = getRobotWidth().value();
        } catch (std::runtime_error &) {
            // Then getRobotWidth() isn't implemented
        }

        std::stringstream ss;
        ss << "TNK_PARAMS " << maxTurnSpeed << " " << maxForwardSpeed << " " << axisLength << "\n";
        connection.getSocketWriter().send(ss.str());

        // Handle incoming TNK commands
        connection.setCommandHandler("TNK",
            [this](Net::Connection &connection, const Net::Command &command) {
                onCommandReceived(connection, command);
            });

        m_Connection = &connection;
    }

    void stopReadingFromNetwork()
    {
        if (m_Connection) {
            // Ignore incoming TNK commands
            m_Connection->setCommandHandler("TNK", nullptr);
        }
    }

private:
    Net::Connection *m_Connection = nullptr;
    float m_X = 0;
    float m_Y = 0;

    void drive(float x, float y, float deadZone)
    {
        const float pi = 3.141592653589793238462643383279502884f;
        const float halfPi = pi / 2.0f;

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
            r = std::min(1.f, r);
            const float theta = atan2(x, -y);
            const float twoTheta = 2.0f * theta;

            // Drive motor
            if (theta >= 0.0f && theta < halfPi) {
                tank(r, r * cos(twoTheta));
            } else if (theta >= halfPi && theta < pi) {
                tank(-r * cos(twoTheta), -r);
            } else if (theta < 0.0f && theta >= -halfPi) {
                tank(r * cos(twoTheta), r);
            } else if (theta < -halfPi && theta >= -pi) {
                tank(-r, -r * cos(twoTheta));
            }
        }
    }

    void onCommandReceived(Net::Connection &, const Net::Command &command)
    {
        // second space separates left and right parameters
        if (command.size() != 3) {
            throw Net::BadCommandError();
        }

        // parse strings to floats
        const float left = stof(command[1]);
        const float right = stof(command[2]);

        // send motor command
        tank(left, right);
    }

    bool onJoystickEvent(HID::JAxis axis, float value, float deadZone)
    {
        // only interested in left joystick
        switch (axis) {
        case HID::JAxis::LeftStickVertical:
            m_Y = value;
            break;
        case HID::JAxis::LeftStickHorizontal:
            m_X = value;
            break;
        default:
            return false;
        }

        // drive robot with joystick
        drive(m_X, m_Y, deadZone);
        return true;
    }
}; // Tank
} // Robots
} // BoBRobotics
