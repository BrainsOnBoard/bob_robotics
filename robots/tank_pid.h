#pragma once

// BoB robotics includes
#include "../common/circstat.h"
#include "../common/pid.h"
#include "../common/pose.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

template<typename PoseType>
auto
getStraightLineEquation(const PoseType &pose)
{
    struct Params
    {
        double m;
        typename PoseType::LengthType c;

        bool isVertical() const
        {
            return std::isinf(m);
        }
    } p;

    const auto angle = normaliseAngle180(pose.yaw());
    if (angle == 0_deg || angle == 180_deg) {
        p.m = std::numeric_limits<double>::infinity();
        p.c = typename PoseType::LengthType{ std::numeric_limits<double>::signaling_NaN() };
    } else {
        p.m = units::math::tan(angle + 90_deg);
        p.c = pose.y() - p.m * pose.x();
    }
    return p;
}

template<typename PoseType, typename PoseType2>
auto
getVelocities(const PoseType &lastPose,
              const PoseType2 &currentPose,
              const units::time::second_t elapsed,
              const units::length::meter_t width)
{
    using namespace units::math;
    using namespace units::angle;
    using namespace units::length;
    using namespace units::angular_velocity;
    using namespace units::velocity;

    const auto lineLast = getStraightLineEquation(lastPose);
    const auto lineCurrent = getStraightLineEquation(currentPose);
    Vector2<meter_t> centre;
    if (lineLast.isVertical()) {
        centre.x() = lastPose.x();
        centre.y() = lineCurrent.m * centre.x() + lineCurrent.c;
    } else if (lineCurrent.isVertical()) {
        centre.x() = currentPose.x();
        centre.y() = lineLast.m * centre.x() + lineLast.c;
    } else {
        centre.x() = (lineCurrent.c - lineLast.c) / (lineLast.m - lineCurrent.m);
        centre.y() = lineLast.m * centre.x() + lineLast.c;
    }

    const radian_t currentAngle = atan2(currentPose.y() - centre.y(), currentPose.x() - centre.x());
    const radian_t lastAngle = atan2(lastPose.y() - centre.y(), lastPose.x() - centre.x());
    const radian_t dtheta = circularDistance(currentAngle, lastAngle);
    const radians_per_second_t angularVelocity = dtheta / elapsed;

    const meter_t radius = centre.distance2D(currentPose);

    meters_per_second_t velocity, velocityLeft, velocityRight;
    if (lineLast.m == lineCurrent.m) { // Straight line
        velocity = currentPose.distance2D(lastPose) / elapsed;
        velocityLeft = velocity;
        velocityRight = velocity;
    } else {
        velocity = abs(meters_per_second_t{ radius.value() * angularVelocity.value() });
        velocityLeft = meters_per_second_t{ abs(angularVelocity.value()) * (radius + width / 2).value() };
        velocityRight = meters_per_second_t{ abs(angularVelocity.value()) * (radius - width / 2).value() };
    }

    const radian_t ang = atan2(lastPose.y() - currentPose.y(), lastPose.x() - currentPose.x());
    if (abs(circularDistance(ang, currentPose.yaw())) < 90_deg) {
        velocity = -velocity;
        velocityLeft = -velocityLeft;
        velocityRight = -velocityRight;
    }

    return std::make_tuple(velocity, velocityLeft, velocityRight, angularVelocity, centre, radius);
}

template<typename BaseTankType>
class TankPID
  : public BaseTankType
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    using second_t = units::time::second_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
    template<typename... Ts>
    TankPID(float kp, float ki, float kd, Ts &&... baseTankArgs)
      : BaseTankType(std::forward<Ts>(baseTankArgs)...)
      , m_LeftWheelPID(kp, ki, kd, -.8f, .8f)
      , m_RightWheelPID(kp, ki, kd, -.8f, .8f)
    {}

    void start()
    {
        m_LeftWheelPID.initialise(1.f, 0.1f);
        m_RightWheelPID.initialise(1.f, 0.1f);
    }

    template<typename PoseType>
    auto updatePose(const PoseType &pose, second_t elapsed)
    {
        meters_per_second_t velocity;
        degrees_per_second_t angularVelocity;
        Vector2<meter_t> centre;
        meter_t radius;
        std::tie(velocity, m_CurrentLeft, m_CurrentRight, angularVelocity, centre, radius) = getVelocities(pose, m_LastPose, elapsed, this->getRobotWidth());
        m_LastPose = pose;

        updateMotors();

        return -m_CurrentLeft;
    }

    virtual void controlWithThumbsticks(HID::Joystick &joystick) override
    {
        joystick.addHandler(
                [this](HID::JAxis axis, float value) {
                    static meters_per_second_t left{}, right{};

                    if (!m_DriveWithVelocities) {
                        return this->drive(axis, value);
                    }

                    switch (axis) {
                    case HID::JAxis::LeftStickVertical:
                        left = -value * this->getMaximumSpeed();
                        break;
                    case HID::JAxis::RightStickVertical:
                        right = -value * this->getMaximumSpeed();
                        break;
                    default:
                        return false;
                    }

                    tankVelocities(left, right);
                    return true;
                });
    }

    void setDriveWithVelocities(const bool flag)
    {
        m_DriveWithVelocities = flag;
    }

    virtual void tankVelocities(meters_per_second_t left,
                                meters_per_second_t right) override
    {
        m_LeftSetPoint = left;
        m_RightSetPoint = right;
        updateMotors();
    }

private:
    Pose2<meter_t, radian_t> m_LastPose;
    PID m_LeftWheelPID, m_RightWheelPID;
    meters_per_second_t m_CurrentLeft, m_CurrentRight, m_LeftSetPoint, m_RightSetPoint;
    bool m_DriveWithVelocities = false;

    void updateMotors()
    {
        // std::cout << "target: " << m_LeftSetPoint << " | current: " << m_CurrentLeft << std::endl;
        float leftMotor = m_LeftWheelPID.update(m_LeftSetPoint.value(), -m_CurrentLeft.value(), true);
        float rightMotor = m_RightWheelPID.update(m_RightSetPoint.value(), -m_CurrentRight.value(), false);
        if (leftMotor > 0.f) {
            leftMotor += 0.2;
        } else {
            leftMotor -= 0.2;
        }
        if (rightMotor > 0.f) {
            rightMotor += 0.2;
        } else {
            rightMotor -= 0.2;
        }
        BaseTankType::tank(leftMotor, rightMotor);
    }
}; // TankPID
} // Robots
} // BoBRobotics