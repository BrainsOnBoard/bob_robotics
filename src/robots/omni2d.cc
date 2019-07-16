// BoB robotics includes
#include "robots/omni2d.h"
#include "common/logging.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Robots {

void
Omni2D::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](HID::JAxis axis, float value) {
                return onJoystickEvent(axis, value, deadZone);
            });
}

void
Omni2D::drive(const HID::Joystick &joystick, float deadZone)
{
    drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
          joystick.getState(HID::JAxis::LeftStickVertical),
          joystick.getState(HID::JAxis::RightStickHorizontal),
          deadZone);
}

void
Omni2D::omni2D(float forward, float sideways, float turn)
{
    LOGI << "Dummy motor: forward: " << forward << "; sideways: " << sideways << "; turn: " << turn;
}

void
Omni2D::readFromNetwork(Net::Connection &connection)
{
    // handle incoming TNK commands
    connection.setCommandHandler("OMN", [this](Net::Connection &connection, const Net::Command &command) {
        onCommandReceived(connection, command);
    });
}

void
Omni2D::drive(float x, float y, float rot, float deadZone)
{
    const bool deadX = (fabs(x) < deadZone);
    const bool deadY = (fabs(y) < deadZone);
    const bool deadRot = (fabs(rot) < deadZone);

    // Drive motor
    omni2D(x * !deadX, y * !deadY, rot * !deadRot);
}

void
Omni2D::onCommandReceived(Net::Connection &, const Net::Command &command)
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

bool
Omni2D::onJoystickEvent(HID::JAxis axis, float value, float deadZone)
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
} // Robots
} // BoBRobotics
