// BoB robotics includes
#include "common/logging.h"
#include "robots/omni2d.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Robots {

Omni2D::~Omni2D()
{
    stopReadingFromNetwork();
    stopMoving();
}

void
Omni2D::omni2D(float forward, float sideways, float turn)
{
    LOGI << "Dummy motor: forward: " << forward << "; sideways: " << sideways << "; turn: " << turn;
}

void Omni2D::moveForward(float speed)
{
    omni2D(speed, 0.0f, 0.0f);
}

void Omni2D::turnOnTheSpot(float clockwiseSpeed)
{
    omni2D(0.0f, 0.0f, clockwiseSpeed);
}

void Omni2D::stopMoving()
{
    omni2D(0.0f, 0.0f, 0.0f);
}

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
    drive(-joystick.getState(HID::JAxis::LeftStickVertical),
          joystick.getState(HID::JAxis::LeftStickHorizontal),
          joystick.getState(HID::JAxis::RightStickHorizontal),
          deadZone);
}


void
Omni2D::readFromNetwork(Net::Connection &connection)
{
    // handle incoming TNK commands
    connection.setCommandHandler("OMN", [this](Net::Connection &connection, const Net::Command &command) {
        onCommandReceived(connection, command);
    });
    
    m_Connection = &connection;
}

void Omni2D::stopReadingFromNetwork()
{
    if (m_Connection) {
        // Ignore incoming TNK commands
        m_Connection->setCommandHandler("TNK", nullptr);
    }
}


void
Omni2D::drive(float forward, float sideways, float turn, float deadZone)
{
    const bool deadForward = (fabs(forward) < deadZone);
    const bool deadSideways = (fabs(sideways) < deadZone);
    const bool deadTurn = (fabs(turn) < deadZone);

    // Drive motor
    omni2D(forward * !deadForward, sideways * !deadSideways, turn * !deadTurn);
}

void
Omni2D::onCommandReceived(Net::Connection &, const Net::Command &command)
{
    // second space separates left and right parameters
    if (command.size() != 4) {
        throw Net::BadCommandError();
    }

    // parse strings to floats
    const float forward = stof(command[1]);
    const float sideways = stof(command[2]);
    const float turn = stof(command[3]);

    // send motor command
    omni2D(forward, sideways, turn);
}

bool
Omni2D::onJoystickEvent(HID::JAxis axis, float value, float deadZone)
{
    float forward = m_Forward;
    float sideways = m_Sideways;
    float turn = m_Turn;
    switch (axis) {
    case HID::JAxis::LeftStickVertical:
        forward = -value;
        break;
    case HID::JAxis::LeftStickHorizontal:
        sideways = value;
        break;
    case HID::JAxis::RightStickHorizontal:
        turn = value;
        break;
    default:
        return false;
    }

    // drive robot with joystick
    drive(forward, sideways, turn, deadZone);
    return true;
}
} // Robots
} // BoBRobotics
