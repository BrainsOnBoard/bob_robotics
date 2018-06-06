#pragma once

// C++ includes
#include <string>

// GeNN robotics includes
#include "../hid/joystick.h"
#include "../net/node.h"

namespace GeNNRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// Motor
//----------------------------------------------------------------------------
// Interface for driving tank-like wheeled robots
class Motor
{
public:
    virtual ~Motor()
    {}

    void addJoystick(HID::Joystick &joystick)
    {
        joystick.addHandler([this](HID::JAxis axis, float value) { return onJoystickEvent(axis, value); });
    }

    virtual void tank(float left, float right)
    {
        std::cout << "Dummy motor: left: " << left << "; right: " << right
                  << std::endl;
    }

    void readFromNetwork(Net::Node &node)
    {
        // handle incoming TNK commands
        node.addCommandHandler("TNK", [this] (Net::Node &node, const Net::Command &command) {
            onCommandReceived(node, command);
        });
    }

private:
    float m_X = 0;
    float m_Y = 0;

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

        // If joystick is in dead zone, stop robot
        if (x == 0 && y == 0) {
            tank(0.0f, 0.0f);
            return true;
        }

        const float r = sqrt((x * x) + (y * y));
        const float theta = atan2(x, -y);
        const float twoTheta = 2.0f * theta;

        // Drive motor
        const float pi = 3.141592653589793238462643383279502884f;
        if (theta >= 0.0f && theta < pi / 2) {
            tank(r, r * cos(twoTheta));
        } else if (theta >= pi / 2 && theta < pi) {
            tank(-r * cos(twoTheta), -r);
        } else if (theta < 0.0f && theta >= -pi / 2) {
            tank(r * cos(twoTheta), r);
        } else if (theta < -pi / 2 && theta >= -pi) {
            tank(-r, -r * cos(twoTheta));
        }

        // signal that we have handled the event
        return true;
    }
}; // Motor
} // Robots
} // GeNNRobotics
