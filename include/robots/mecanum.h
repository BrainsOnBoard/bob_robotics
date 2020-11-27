#ifdef __linux__
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
    Mecanum(const char *path = "/dev/ttyACM0", bool alternativeWiring = true);
    virtual ~Mecanum();
    
    //----------------------------------------------------------------------------
    // Omni2D virtuals
    //----------------------------------------------------------------------------
    virtual void omni2D(float forward, float sideways, float turn) override;

    void driveMotors(float m1, float m2, float m3, float m4);
    
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

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::SerialInterface m_Serial;
    const bool m_AlternativeWiring;
}; // Mecanum
} // Robots
} // BoBRobotics
#endif	// __linux__
