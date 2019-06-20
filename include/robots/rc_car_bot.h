#pragma once

// BoB robotics includes
#include "common/i2c_interface.h"
#include "third_party/units.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::RCCarBot
//----------------------------------------------------------------------------
//! An interface for 4 wheeled, Arduino-based robots developed at the University of Sussex
class RCCarBot
{
    using degree_t = units::angle::degree_t;

public:
    RCCarBot(const char *path = "/dev/i2c-1", int slaveAddress = 0x29);
    ~RCCarBot();

    //! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
    void move(float speed, degree_t turningAngle);

    //! Stop the car
    void stopMoving();
    float getSpeed() const;
    degree_t getTurningAngle() const;
    degree_t getMaximumTurn() const;

private:
    BoBRobotics::I2CInterface m_I2C; // i2c interface
    float m_speed;                   // current control speed of the robot
    degree_t m_turningAngle;         // current turning angle of the robot

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }

}; // RCCarBot
} // Robots
} // BoBRobotics
