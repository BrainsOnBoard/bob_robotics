#pragma once

// C++ includes
#include <string>

// GeNN robotics includes
#include "../hid/joystick.h"
#include "../net/node.h"

namespace GeNNRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// GeNNRobotics::Robots::Tank
//----------------------------------------------------------------------------
// Interface for driving tank-like wheeled robots
class Tank
{
public:
    virtual ~Tank()
    {}

    void addJoystick(HID::Joystick &joystick, float deadZone = 0.25f)
    {
        m_DeadZone = deadZone;
        joystick.addHandler([this](HID::JAxis axis, float value) { return onJoystickEvent(axis, value); });
    }

    void drive(HID::Joystick &joystick, float deadZone = m_DeadZone)
    {
        drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
              joystick.getState(HID::JAxis::LeftStickVertical), deadZone);
    }

    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
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
    float m_DeadZone = 0.25f;

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

    void onCommandReceived(Net::Node &node, const Net::Command &command)
    {
        // second space separates left and right parameters
        if (command.size() != 3) {
            throw Net::bad_command_error();
        }

        // parse strings to floats
        const float left = stof(command[1]);
        const float right = stof(command[2]);

        // send motor command
        tank(left, right);
    }

    bool onJoystickEvent(HID::JAxis axis, float value)
    {
        // only interested in left joystick
        float x = m_X;
        float y = m_Y;
        switch (axis) {
        case HID::JAxis::LeftStickVertical:
            y = value;
            break;
        case HID::JAxis::LeftStickHorizontal:
            x = value;
            break;
        default:
            return false;
        }

        // drive robot with joystick
        drive(x, y, m_DeadZone);
        return true;
    }
}; // Tank
} // Robots
} // GeNNRobotics
