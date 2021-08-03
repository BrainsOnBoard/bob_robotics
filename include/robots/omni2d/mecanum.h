#ifdef __linux__
#pragma once

// BoB robotics includes
#include "common/serial_interface.h"
#include "robots/omni2d/omni2d_base.h"

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D::Mecanum
//----------------------------------------------------------------------------
class Mecanum
  : public Omni2DBase<Mecanum>
{
public:
    Mecanum(bool alternativeWiring = true,
            const char *path = SerialInterface::DefaultLinuxDevicePath);
    ~Mecanum();

    void omni2D(float forward, float sideways, float turn);

    //! Drive robot using individual motor speeds - all range from -1 to 1
    void driveMotors(float m1, float m2, float m3, float m4);

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    const bool m_AlternativeWiring;
    SerialInterface m_Serial;

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
}; // Mecanum
} // Omni2D
} // Robots
} // BoBRobotics
#endif	// __linux__
