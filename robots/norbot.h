#pragma once

// BoB robotics includes
#include "../common/i2c_interface.h"
#include "tank.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <vector>

// third party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Norbot
//----------------------------------------------------------------------------
//! An interface for wheeled, Arduino-based robots developed at the University of Sussex
class Norbot : public Tank
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;

public:
    Norbot(const char *path = "/dev/i2c-1", int slaveAddress = 0x29)
      : m_I2C(path, slaveAddress)
    {
        // Sometimes Norbots get stuck driving, so let's stop it if we need to
        stopMoving();
    }

    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
    virtual ~Norbot() override
    {
        stopMoving();
        stopReadingFromNetwork();
    }

    virtual void tank(float left, float right) override
    {
        setWheelSpeeds(left, right);

        // Cache left and right
        const float maxSpeed = getMaximumSpeedProportion();
        left *= maxSpeed;
        right *= maxSpeed;

        // Convert standard (-1,1) values to bytes in order to send to I2C slave
        uint8_t buffer[2] = { floatToI2C(left), floatToI2C(right) };

        // Send buffer
        write(buffer);
    }

    virtual meters_per_second_t getMaximumSpeed() override
    {
        return getMaximumSpeedProportion() * 0.11_mps;
    }

    virtual millimeter_t getRobotWidth() override
    {
        return 104_mm;
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        m_I2C.read(data);
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }

private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    uint8_t floatToI2C(float speed)
    {
        return (uint8_t) std::min(
                255.0f,
                std::max(0.0f, std::round(((speed + 1.0f) / 2.0f) * 255.0f)));
    }

    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::I2CInterface m_I2C;
}; // Norbot
} // Robots
} // BoBRobotics
