#ifdef __linux__
#pragma once

// BoB robotics includes
#include "common/i2c_interface.h"
#include "robots/ackermann.h"
#include "robots/rc_car_common.h"
#include "third_party/units.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::RCCarBot
//----------------------------------------------------------------------------
//! An interface for 4 wheeled, Arduino-based robots developed at the University of Sussex
class RCCarBot final
  : public Ackermann
{
    using degree_t = units::angle::degree_t;

public:
    RCCarBot(const char *path = I2C_DEVICE_DEFAULT);
    virtual ~RCCarBot();

    float getSpeed() const;
    degree_t getTurningAngle() const;

    // Public virtuals
    virtual void moveForward(float speed) override;
    virtual void steer(float left) override;
    virtual void steer(units::angle::degree_t left) override;
    virtual degree_t getMaximumTurn() const override;

    //! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
    virtual void move(float speed, degree_t left) override;

    //! Stop the car
    virtual void stopMoving() override;

    //! Read speed and turn values from remote control
    std::pair<float, degree_t> readRemoteControl();

    constexpr static degree_t TurnMax{ 35 };

private:
    BoBRobotics::I2CInterface m_I2C; // i2c interface
    float m_speed;                   // current control speed of the robot
    degree_t m_turningAngle;         // current turning angle of the robot

    void setState(RCCar::State state);

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }

}; // RCCarBot
} // Robots
} // BoBRobotics
#endif	// __linux__
