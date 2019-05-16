// BoB robotics includes
#include "common/macros.h"
#include "common/circstat.h"
#include "common/logging.h"
#include "robots/tank.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace units::length;
using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {

void Tank::moveForward(float speed)
{
    tank(speed, speed);
}

void Tank::turnOnTheSpot(float clockwiseSpeed)
{
    tank(clockwiseSpeed, -clockwiseSpeed);
}

void Tank::stopMoving()
{
    tank(0.f, 0.f);
}

void Tank::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](HID::JAxis axis, float value) {
                return onJoystickEvent(axis, value, deadZone);
            });
}

void Tank::controlWithThumbsticks(HID::Joystick &joystick)
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

void Tank::drive(const HID::Joystick &joystick, float deadZone)
{
    drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
          joystick.getState(HID::JAxis::LeftStickVertical),
          deadZone);
}

void Tank::move(meters_per_second_t v,
                radians_per_second_t clockwiseSpeed,
                const bool maxScaled)
{
    const meter_t axisLength = getRobotWidth();
    const meters_per_second_t diff{
        (clockwiseSpeed * axisLength / 2).value()
    };
    const meters_per_second_t vL = v + diff;
    const meters_per_second_t vR = v - diff;
    tank(vL, vR, maxScaled);
}

//! Set the left and right motors to the specified speed
void Tank::tank(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);
    LOG_INFO << "Dummy motor: left: " << left << "; right: " << right;
}

void Tank::tankMaxScaled(const float left, const float right, const float max)
{
    const float larger = std::max(std::abs(left), std::abs(right));
    if (larger <= max) {
        tank(left, right);
    } else {
        const float ratio = max / larger;
        tank(ratio * left, ratio * right);
    }
}

void Tank::tank(meters_per_second_t left, meters_per_second_t right, bool maxScaled)
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

millimeter_t Tank::getRobotWidth() const
{
    throw std::runtime_error("getRobotWidth() is not implemented for this class");
}

meters_per_second_t Tank::getMaximumSpeed() const
{
    return getMaximumSpeedProportion() * getAbsoluteMaximumSpeed();
}

meters_per_second_t Tank::getAbsoluteMaximumSpeed() const
{
    throw std::runtime_error("getAbsoluteMaximumSpeed() is not implemented for this class");
}

radians_per_second_t Tank::getMaximumTurnSpeed() const
{
    return getMaximumSpeedProportion() * getAbsoluteMaximumTurnSpeed();
}

radians_per_second_t Tank::getAbsoluteMaximumTurnSpeed() const
{
    // max turn speed = v_max / r
    return radians_per_second_t{ (getAbsoluteMaximumSpeed() * 2 / static_cast<meter_t>(getRobotWidth())).value() };
}

void Tank::setMaximumSpeedProportion(float value)
{
    BOB_ASSERT(value >= -1.f && value <= 1.f);
    m_MaximumSpeedProportion = value;
}

float Tank::getMaximumSpeedProportion() const
{
    return m_MaximumSpeedProportion;
}

//! Controls the robot with a network stream
void Tank::readFromNetwork(Net::Connection &connection)
{
    // Send robot parameters over network
    constexpr double _nan = std::numeric_limits<double>::quiet_NaN();
    double maxTurnSpeed = _nan, maxForwardSpeed = _nan, axisLength = _nan;
    try {
        maxTurnSpeed = getAbsoluteMaximumTurnSpeed().value();
    } catch (std::runtime_error &) {
        // Then getMaximumTurnSpeed() isn't implemented
    }
    try {
        maxForwardSpeed = getAbsoluteMaximumSpeed().value();
    } catch (std::runtime_error &) {
        // Then getMaximumSpeed() isn't implemented
    }
    try {
        axisLength = getRobotWidth().value();
    } catch (std::runtime_error &) {
        // Then getRobotWidth() isn't implemented
    }

    std::stringstream ss;
    ss << "TNK_PARAMS "
        << maxTurnSpeed << " "
        << maxForwardSpeed << " "
        << axisLength << " "
        << getMaximumSpeedProportion() << "\n";
    connection.getSocketWriter().send(ss.str());

    // Handle incoming TNK commands
    connection.setCommandHandler("TNK",
                                    [this](Net::Connection &connection, const Net::Command &command) {
                                        onCommandReceived(connection, command);
                                    });

    connection.setCommandHandler("TNK_MAX",
                                    [this](Net::Connection &, const Net::Command &command) {
                                        Tank::setMaximumSpeedProportion(stof(command.at(1)));
                                        tank(m_Left, m_Right);
                                    });

    m_Connection = &connection;
}

void Tank::stopReadingFromNetwork()
{
    if (m_Connection) {
        // Ignore incoming TNK commands
        m_Connection->setCommandHandler("TNK", nullptr);
    }
}

void Tank::drive(float x, float y, float deadZone)
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

void Tank::onCommandReceived(Net::Connection &, const Net::Command &command)
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

bool Tank::onJoystickEvent(HID::JAxis axis, float value, float deadZone)
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

float Tank::getLeft() const
{
    return m_Left;
}

float Tank::getRight() const
{
    return m_Right;
}

void Tank::setWheelSpeeds(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);

    m_Left = left;
    m_Right = right;
}

} // Robots
} // BoBRobotics
