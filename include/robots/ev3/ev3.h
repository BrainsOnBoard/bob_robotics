#pragma once

// BoB robotics includes
#include "common/stopwatch.h"
#include "robots/tank.h"

// EV3 library
#include "../../third_party/ev3dev-lang-cpp/ev3dev.h"

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
    /*
     * BrickPis are hardcoded to treat all motors as NXT motors, so we pretend
     * our large Lego motors are NXT motors, otherwise ev3dev-lang-cpp will give
     * an error.
     */
#if defined(EV3DEV_PLATFORM_BRICKPI) || defined(EV3DEV_PLATFORM_BRICKPI3)
    using MotorType = ev3dev::nxt_motor;
#else
    using MotorType = ev3dev::large_motor;
#endif

    EV3(const ev3dev::address_type leftMotorPort = ev3dev::OUTPUT_A,
        const ev3dev::address_type rightMotorPort = ev3dev::OUTPUT_D);
    virtual ~EV3() override;

    virtual void stopMoving() override;
    virtual void tank(float left, float right) override;
    virtual millimeter_t getRobotWidth() const override;
    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;
    std::pair<meters_per_second_t, meters_per_second_t> getWheelVelocities() const;

private:
    MotorType m_MotorLeft, m_MotorRight;
    Stopwatch m_MotorStatusTimer;
    const int m_MaxSpeedTachos, m_TachoCountPerRotation;

    meters_per_second_t tachoToSpeed(int tachos) const;
};
}
}
