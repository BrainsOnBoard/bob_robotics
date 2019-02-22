#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "tank.h"

// EV3 library
#include "ev3dev.h"

namespace BoBRobotics {
namespace Robots {
//! Control for a Lego Mindstorms tank robot
class EV3
  : public Tank
{
public:
    EV3(const ev3dev::address_type leftMotorPort = ev3dev::OUTPUT_D,
        const ev3dev::address_type rightMotorPort = ev3dev::OUTPUT_A)
      : m_MotorLeft(leftMotorPort)
      , m_MotorRight(rightMotorPort)
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

        m_MotorLeft.set_speed_sp(700.f * left);
        m_MotorRight.set_speed_sp(700.f * right);
        m_MotorLeft.run_forever();
        m_MotorRight.run_forever();
    }

private:
    ev3dev::large_motor m_MotorLeft, m_MotorRight;
};
}
}