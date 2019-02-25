#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/circstat.h"
#include "tank.h"

// EV3 library
#include "ev3dev.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
//! Control for a Lego Mindstorms tank robot
class EV3
  : public Tank
{
public:
    EV3(const float proportionMaxSpeed = 0.7f,
        const ev3dev::address_type leftMotorPort = ev3dev::OUTPUT_D,
        const ev3dev::address_type rightMotorPort = ev3dev::OUTPUT_A)
      : m_MotorLeft(leftMotorPort)
      , m_MotorRight(rightMotorPort)
      , m_MaxSpeed(proportionMaxSpeed * m_MotorLeft.max_speed())
    {}

    virtual ~EV3() override
    {
        stopMoving();
    }

    virtual void stopMoving() override
    {
        m_MotorLeft.stop();
        m_MotorRight.stop();
    }

    virtual void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);

        m_MotorLeft.set_speed_sp(m_MaxSpeed * left);
        m_MotorRight.set_speed_sp(m_MaxSpeed * right);
        m_MotorLeft.run_forever();
        m_MotorRight.run_forever();
    }

    virtual millimeter_t getRobotWidth() override
    {
        return 12_cm;
    }

    virtual meters_per_second_t getMaximumSpeed() override
    {
        constexpr units::length::meter_t wheelRadius = 5.5_cm / 2;
        const double turnsPerTacho = 1.0 / m_MotorLeft.count_per_rot();
        const double angularVelocity{ m_MaxSpeed * 2 * pi() * turnsPerTacho };
        return meters_per_second_t{ angularVelocity * wheelRadius.value() };
    }

private:
    ev3dev::large_motor m_MotorLeft, m_MotorRight;
    const float m_MaxSpeed;
};
}
}