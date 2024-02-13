// BoB robotics includes
#include "antworld/agent.h"
#include "common/circstat.h"
#include "common/macros.h"

using namespace units::literals;
using namespace units::velocity;
using namespace units::angular_velocity;

namespace BoBRobotics {
namespace AntWorld {

// Declarations
meters_per_second_t AntAgent::DefaultVelocity = 0.03_mps;
radians_per_second_t AntAgent::DefaultTurnSpeed = 200_deg_per_s;

AntAgent::AntAgent(sf::Window &window,
                   Renderer &renderer,
                   const cv::Size &renderSize,
                   meters_per_second_t velocity,
                   radians_per_second_t turnSpeed)
  : Camera(window, renderer, renderSize)
  , m_Velocity(velocity)
  , m_TurnSpeed(turnSpeed)
{}

void
AntAgent::moveForward(float speed)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);
    const auto attitude = getPose().attitude();
    BOB_ASSERT(attitude[1] == 0_deg && attitude[2] == 0_deg);

    m_MoveMode = MoveMode::MovingForward;
    m_SpeedProportion = speed;
}

void
AntAgent::turnOnTheSpot(float clockwiseSpeed)
{
    BOB_ASSERT(clockwiseSpeed >= -1.f && clockwiseSpeed <= 1.f);
    const auto attitude = getPose().attitude();
    BOB_ASSERT(attitude[1] == 0_deg && attitude[2] == 0_deg);

    m_MoveMode = MoveMode::Turning;
    m_SpeedProportion = clockwiseSpeed;
}

void
AntAgent::stopMoving()
{
    m_MoveMode = MoveMode::NotMoving;
}

void
AntAgent::addJoystick(HID::Joystick &joystick, float deadZone)
{
    joystick.addHandler(
            [this, deadZone](auto &, HID::JAxis axis, float value) {
                switch (axis) {
                case HID::JAxis::LeftStickVertical:
                    m_JoystickY = value;
                    break;
                case HID::JAxis::LeftStickHorizontal:
                    m_JoystickX = value;
                    break;
                default:
                    return false;
                }

                if (fabs(m_JoystickY) > deadZone) {
                    moveForward(-m_JoystickY);
                } else if (fabs(m_JoystickX) > deadZone) {
                    turnOnTheSpot(m_JoystickX);
                } else {
                    stopMoving();
                }
                return true;
            });
}

void AntAgent::drive(const HID::Joystick &joystick, float deadZone)
{
    m_JoystickX = joystick.getState(HID::JAxis::LeftStickHorizontal);
    m_JoystickY = joystick.getState(HID::JAxis::LeftStickVertical);

    if (fabs(m_JoystickY) > deadZone) {
        moveForward(-m_JoystickY);
    } else if (fabs(m_JoystickX) > deadZone) {
        turnOnTheSpot(m_JoystickX);
    } else {
        stopMoving();
    }
}

radians_per_second_t
AntAgent::getMaximumTurnSpeed() const
{
    return m_TurnSpeed;
}

void
AntAgent::setPosition(meter_t x, meter_t y, meter_t z)
{
    BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);
    Camera::setPosition(x, y, z);
}

void
AntAgent::setAttitude(degree_t yaw, degree_t pitch, degree_t roll)
{
    BOB_ASSERT(m_MoveMode == MoveMode::NotMoving);
    Camera::setAttitude(yaw, pitch, roll);
}


void
AntAgent::updatePose(const units::time::second_t elapsedTime)
{
    std::lock_guard<std::mutex> guard(m_PoseMutex);
    const auto pose = getPose();

    using namespace units::math;
    switch (m_MoveMode) {
    case MoveMode::MovingForward: {
        const meter_t dist = m_SpeedProportion * m_Velocity * elapsedTime;
        meter_t x = pose.x(), y = pose.y();
        x += dist * cos(pose.yaw());
        y += dist * sin(pose.yaw());
        Camera::setPosition(x, y, pose.z());
    } break;
    case MoveMode::Turning: {
        degree_t yaw = pose.yaw();
        yaw -= m_SpeedProportion * m_TurnSpeed * elapsedTime;
        yaw = normaliseAngle360(yaw);
        Camera::setAttitude(yaw, degree_t{ 0 }, degree_t{ 0 });
    } break;
    default:
        break;
    }
}

} // AntWorld
} // BoBRobotics
