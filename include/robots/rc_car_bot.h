#pragma once
// BoB robotics includes
#include "../common/assert.h"
#include "../common/i2c_interface.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <vector>

// third party includes
#include "../third_party/units.h"

using namespace units::literals;

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::RCCarBot
//----------------------------------------------------------------------------
//! An interface for 4 wheeled, Arduino-based robots developed at the University of Sussex
class RCCarBot
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;
    using degree_t = units::angle::degree_t;

public:
    RCCarBot(const char *path = "/dev/i2c-1", int slaveAddress = 0x29)
      : m_I2C(path, slaveAddress)
      , m_speed(0.0f)
      , m_turningAngle(0_deg)
    {}

    ~RCCarBot()
    {
        stopMoving();
    }

    //! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
    void move(float speed, degree_t turningAngle)
    {
        BOB_ASSERT(speed >= -1.f && speed <= 1.f);
        BOB_ASSERT(turningAngle >= -35_deg && turningAngle <= 35_deg);

        m_speed = speed;
        m_turningAngle = m_turningAngle;

        // mapping to the range
        uint8_t uspeed = (speed * 255) + sgn(-speed) * 127;     // mapping to : 0-127 backward, 127-255 forward
        uint8_t uturn = (uint8_t)(90.0 + turningAngle.value()); // mapping to : 65 full left 90 center 125 full right
        uint8_t buffer[2] = { uspeed, uturn };
        m_I2C.write(buffer); // send to arduino on i2c
    }

    // stops the car
    void stopMoving()
    {
        move(0, 0_deg);
    }

    float getSpeed() const
    {
        return m_speed;
    }

    degree_t getTurningAngle() const
    {
        return m_turningAngle;
    }

private:
    BoBRobotics::I2CInterface m_I2C; // i2c interface
    float m_speed;                   // current control speed of the robot
    degree_t m_turningAngle;         // current turning angle of the robot

    // sign function
    template<typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }

}; // RCCarBot
} // Robots
} // BoBRobotics
