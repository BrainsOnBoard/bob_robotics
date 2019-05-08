#pragma once

// BoB robotics includes
#include "robots/tank.h"

// EV3 library
#include "ev3dev.h"

// Standard C++ includes
#include <string>
#include <utility>

namespace BoBRobotics {
namespace Robots {
//! Control for a Lego Mindstorms tank robot
class EV3
  : public Tank
{
public:
    EV3(const ev3dev::address_type leftMotorPort = ev3dev::OUTPUT_A,
        const ev3dev::address_type rightMotorPort = ev3dev::OUTPUT_D);
    virtual ~EV3() override;

    virtual void stopMoving() override;
    virtual void tank(float left, float right) override;
    virtual millimeter_t getRobotWidth() const override;
    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;
    std::pair<meters_per_second_t, meters_per_second_t> getWheelVelocities() const;

private:
    ev3dev::large_motor m_MotorLeft, m_MotorRight;
    const int m_MaxSpeedTachos, m_TachoCountPerRotation;

    meters_per_second_t tachoToSpeed(int tachos) const;
};
}
}
