#pragma once

// C++ includes
#include <string>

// BoB robotics includes
#include "hid/joystick.h"
#include "net/connection.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D
//----------------------------------------------------------------------------
// Interface for driving Omni2D-like wheeled robots
class Omni2D
{
public:
    virtual ~Omni2D()
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
              joystick.getState(HID::JAxis::LeftStickVertical),joystick.getState(HID::JAxis::RightStickHorizontal), deadZone);
    }

    virtual void omni2D(float forward, float sideways, float turn)
    {
        std::cout << "Dummy motor: forward: " << forward << "; sideways: " << sideways << "; turn: " << turn << std::endl;
    }

    void readFromNetwork(Net::Connection &connection)
    {
        // handle incoming TNK commands
        connection.setCommandHandler("OMN", [this](Net::Connection &connection, const Net::Command &command) {
            onCommandReceived(connection, command);
        });
    }

private:
    float m_X = 0;
    float m_Y = 0;
    float m_R = 0;

    void drive(float x, float y, float rot, float deadZone)
    {
        const float pi = 3.141592653589793238462643383279502884f;
        const float halfPi = pi / 2.0f;

        const bool deadX = (fabs(x) < deadZone);
        const bool deadY = (fabs(y) < deadZone);
        const bool deadRot = (fabs(rot) < deadZone);

        // If length of joystick vector places it in deadZone, stop motors
        //const float r = sqrt((x * x) + (y * y));
        //const float theta = atan2(x, -y);
        //const float twoTheta = 2.0f * theta;

        // Drive motor
        omni2D(x*!deadX, y*!deadY, rot*!deadRot);

    }

    void onCommandReceived(Net::Connection &, const Net::Command &command)
    {
        // second space separates left and right parameters
        if (command.size() != 4) {
            throw Net::BadCommandError();
        }

        // parse strings to floats
        const float left = stof(command[1]);
        const float right = stof(command[2]);
        const float turn = stof(command[3]);

        // send motor command
        omni2D(left, right, turn);
    }

    bool onJoystickEvent(HID::JAxis axis, float value, float deadZone)
    {
        float x = m_X;
        float y = m_Y;
        float rot = m_R;
        switch (axis) {
        case HID::JAxis::LeftStickVertical:
            y = value;
            break;
        case HID::JAxis::LeftStickHorizontal:
            x = value;
            break;
        case HID::JAxis::RightStickHorizontal:
            rot = value;
            break;
        default:
            return false;
        }

        // drive robot with joystick
        drive(x, y, rot, deadZone);
        return true;
    }
}; // Omni2D
} // Robots
} // BoBRobotics
