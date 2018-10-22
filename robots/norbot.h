#pragma once

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

// Common includes
#include "../common/i2c_interface.h"
#include "tank.h"

// third party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Norbot
//----------------------------------------------------------------------------
//! An interface for wheeled, Arduino-based robots developed at the University of Sussex
class Norbot : public Tank
{
using millimeter_t = units::length::millimeter_t;

public:
    Norbot(const char *path = "/dev/i2c-1", int slaveAddress = 0x29)
      : m_I2C(path, slaveAddress)
      , m_Left(0.0f)
      , m_Right(0.0f)
    {}

    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
    virtual void tank(float left, float right) override
    {
        // Cache left and right
        m_Left = left;
        m_Right = right;

        // Convert standard (-1,1) values to bytes in order to send to I2C slave
        uint8_t buffer[2] = { floatToI2C(left), floatToI2C(right) };

        // Send buffer
        write(buffer);
    }

    virtual millimeter_t getRobotWheelRadius() override
    {
        return WheelRadius;
    }

    virtual millimeter_t getRobotAxisLength() override
    {
        return AxisLength;
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

    float getLeft() const
    {
        return m_Left;
    }

    float getRight() const
    {
        return m_Right;
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
    float m_Left;
    float m_Right;

    static constexpr millimeter_t WheelRadius{ 34 };
    static constexpr millimeter_t AxisLength{ 104 };
}; // Norbot
} // Robots
} // BoBRobotics
