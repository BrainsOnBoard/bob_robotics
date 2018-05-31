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
        joystick.addHandler([this] (HID::Event &js) { return onJoystickEvent(js); });
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

    bool onJoystickEvent(HID::Event &js)
    {
        // only interested in left joystick
        float x = m_X;
        float y = m_Y;
        switch (js.axis()) {
        case HID::Axis::LeftStickVertical:
            y = js.axisValue();
            break;
        case HID::Axis::LeftStickHorizontal:
            x = js.axisValue();
            break;
        default:
            return false;
        }

        // Code below is adapted from Jamie's joystick.h - AD
        // If length of joystick vector places it in deadzone, stop motors
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
