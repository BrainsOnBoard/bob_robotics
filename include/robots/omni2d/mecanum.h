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
    Mecanum(bool alternativeWiring = true,
            const char *path = SerialInterface::DefaultLinuxDevicePath);
    virtual ~Mecanum();

    //----------------------------------------------------------------------------
    // Omni2D virtuals
    //----------------------------------------------------------------------------
    virtual void omni2D(float forward, float sideways, float turn) override;

    //! Drive robot using individual motor speeds - all range from -1 to 1
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
    const bool m_AlternativeWiring;
    SerialInterface m_Serial;
}; // Mecanum
} // Robots
} // BoBRobotics
#endif	// __linux__
