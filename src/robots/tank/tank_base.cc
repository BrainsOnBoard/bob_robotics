// BoB robotics includes
#include "common/macros.h"
#include "common/circstat.h"
#include "robots/tank/tank_base.h"

// Third-party includes
#include "plog/Log.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>

using namespace units::length;
using namespace units::angular_velocity;
using namespace units::velocity;

namespace BoBRobotics {
namespace Robots {
namespace Tank {

TankBase::~TankBase()
{}

void TankBase::moveForward(float speed)
{
    tank(speed, speed);
}

void TankBase::turnOnTheSpot(float clockwiseSpeed)
{
    tank(clockwiseSpeed, -clockwiseSpeed);
}

void TankBase::stopMoving()
{
    tank(0.f, 0.f);
}

void TankBase::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](HID::JAxis axis, float value) {
                return onJoystickEvent(axis, value, deadZone);
            });
}

void TankBase::drive(const HID::Joystick &joystick, float deadZone)
{
    drive(joystick.getState(HID::JAxis::LeftStickHorizontal),
          joystick.getState(HID::JAxis::LeftStickVertical),
          deadZone);
}


//! Controls the robot with a network stream
void TankBase::readFromNetwork(Net::Connection &connection)
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
                                        onTankCommandReceived(connection, command);
                                    });

    connection.setCommandHandler("TNK_MAX",
                                    [this](Net::Connection &, const Net::Command &command) {
                                        TankBase::setMaximumSpeedProportion(stof(command.at(1)));
                                        tank(m_Left, m_Right);
                                    });

    m_Connection = &connection;
}

void TankBase::stopReadingFromNetwork()
{
    if (m_Connection) {
        // Ignore incoming TNK commands
        m_Connection->setCommandHandler("TNK", nullptr);
    }
}

void TankBase::controlWithThumbsticks(HID::JoystickBase<HID::JAxis, HID::JButton> &joystick)
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

void TankBase::move(meters_per_second_t v,
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
void TankBase::tank(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);
    LOG_INFO << "Dummy motor: left: " << left << "; right: " << right;
}

void TankBase::tankMaxScaled(const float left, const float right, const float max)
{
    const float larger = std::max(std::fabs(left), std::fabs(right));
    if (larger <= max) {
        tank(left, right);
    } else {
        const float ratio = max / larger;
        tank(ratio * left, ratio * right);
    }
}

void TankBase::tank(meters_per_second_t left, meters_per_second_t right, bool maxScaled)
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

BOB_NOT_IMPLEMENTED(millimeter_t TankBase::getRobotWidth() const)

meters_per_second_t TankBase::getMaximumSpeed() const
{
    return getMaximumSpeedProportion() * getAbsoluteMaximumSpeed();
}

BOB_NOT_IMPLEMENTED(meters_per_second_t TankBase::getAbsoluteMaximumSpeed() const)

radians_per_second_t TankBase::getMaximumTurnSpeed() const
{
    return getMaximumSpeedProportion() * getAbsoluteMaximumTurnSpeed();
}

radians_per_second_t TankBase::getAbsoluteMaximumTurnSpeed() const
{
    // max turn speed = v_max / r
    return radians_per_second_t{ (getAbsoluteMaximumSpeed() * 2 / static_cast<meter_t>(getRobotWidth())).value() };
}

void TankBase::setMaximumSpeedProportion(float value)
{
    BOB_ASSERT(value >= -1.f && value <= 1.f);
    m_MaximumSpeedProportion = value;
}

float TankBase::getMaximumSpeedProportion() const
{
    return m_MaximumSpeedProportion;
}


void TankBase::drive(float x, float y, float deadZone)
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

void TankBase::onTankCommandReceived(Net::Connection &, const Net::Command &command)
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

bool TankBase::onJoystickEvent(HID::JAxis axis, float value, float deadZone)
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

float TankBase::getLeft() const
{
    return m_Left;
}

float TankBase::getRight() const
{
    return m_Right;
}

void TankBase::setWheelSpeeds(float left, float right)
{
    BOB_ASSERT(left >= -1.f && left <= 1.f);
    BOB_ASSERT(right >= -1.f && right <= 1.f);

    m_Left = left;
    m_Right = right;
}

} // Tank
} // Robots
} // BoBRobotics
