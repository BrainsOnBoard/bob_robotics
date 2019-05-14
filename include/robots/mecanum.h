#pragma once

// BoB robotics includes
#include "common/serial_interface.h"
#include "omni2d.h"

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Mecanum
//----------------------------------------------------------------------------
class Mecanum : public Omni2D
{
public:
    Mecanum(const char *path = "/dev/ttyACM0");

    //----------------------------------------------------------------------------
    // Omni2D virtuals
    //----------------------------------------------------------------------------
    virtual void omni2D(float forwards, float sideways, float turn) override;

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        m_Serial.read(data);
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_Serial.write(data);
    }

    float getForwards() const;
    float getSideways() const;
    float getTurn() const;

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::SerialInterface m_Serial;
    float m_Forward;
    float m_Sideways;
    float m_Turn;
}; // Mecanum
} // Robots
} // BoBRobotics
