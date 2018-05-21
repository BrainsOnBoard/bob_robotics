#pragma once

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

// Common includes
#include "../common/i2c_interface.h"
#include "motor.h"

namespace GeNN_Robotics {
namespace Robots {
//----------------------------------------------------------------------------
// MotorI2C
//----------------------------------------------------------------------------
class MotorI2C : public Motor
{
public:
    MotorI2C(const char *path = "/dev/i2c-1", int slaveAddress = 0x29)
      : m_I2C(path, slaveAddress)
      , m_Left(0.0f)
      , m_Right(0.0f)
    {}

    //----------------------------------------------------------------------------
    // Motor virtuals
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
    I2CInterface m_I2C;
    float m_Left;
    float m_Right;
}; // MotorI2C
} // Robots
} // GeNN_Robotics
