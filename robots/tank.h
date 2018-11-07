#pragma once

// C++ includes
#include <stdexcept>
#include <string>

// BoB robotics includes
#include "../hid/joystick.h"
#include "../net/node.h"

// third party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank
//----------------------------------------------------------------------------
//! Interface for driving wheeled robots with tank steering
class Tank
{
public:
    virtual ~Tank()
    {}

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        joystick.addHandler(
            [this, deadZone](HID::JAxis axis, float value)
            {
                return onJoystickEvent(axis, value, deadZone);
            });
    }

    void drive(const HID::Joystick &joystick, float deadZone = 0.25f)
    {
        drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
              joystick.getState(HID::JAxis::LeftStickVertical), deadZone);
    }

    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }

    virtual units::length::millimeter_t getRobotWheelRadius() 
    {
        throw std::runtime_error("getRobotWheelRadius() is not implemented for this class");
    }

    virtual units::length::millimeter_t getRobotAxisLength() 
    {
        throw std::runtime_error("getRobotAxisLength() is not implemented for this class");
    }

    void readFromNetwork(Net::Node &node)
    {
        // handle incoming TNK commands
        node.addCommandHandler("TNK", [this](Net::Node &node, const Net::Command &command) {
            onCommandReceived(node, command);
        });
    }

private:
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
            const float r = sqrt((x * x) + (y * y));
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

    void onCommandReceived(Net::Node &, const Net::Command &command)
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
