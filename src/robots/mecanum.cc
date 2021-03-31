#ifdef __linux__
// BoB robotics includes
#include "robots/mecanum.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <vector>


namespace BoBRobotics {
namespace Robots {
Mecanum::Mecanum(const char *path, bool alternativeWiring)
  : m_Serial(path), m_AlternativeWiring(alternativeWiring)
{}

Mecanum::~Mecanum()
{
    stopReadingFromNetwork();
    stopMoving();
}
//----------------------------------------------------------------------------
// Omni2D virtuals
//----------------------------------------------------------------------------
void
Mecanum::omni2D(float forward, float sideways, float turn)
{
    // Cache left and right
    setWheelSpeed(forward, sideways, turn);

    // resolve to motor speeds
    const float m1 = m_AlternativeWiring ? (-sideways + forward - turn) : (+sideways - forward - turn);
    const float m2 = +sideways + forward + turn;
    const float m3 = m_AlternativeWiring ? (+sideways + forward - turn) : (-sideways + forward - turn);
    const float m4 = m_AlternativeWiring ? (-sideways + forward + turn) : (-sideways - forward + turn);

    driveMotors(m1, m2, m3, m4);

}
//----------------------------------------------------------------------------
void Mecanum::driveMotors(float m1, float m2, float m3, float m4)
{
    // clamp values to be between -1 and 1 after resolving
    const auto cap = [](float &val) { val = std::min(1.f, std::max(val, -1.f)); };
    cap(m1);
    cap(m2);
    cap(m3);
    cap(m4);

    // **NOTE** 254s are because 255 is line end character
    uint8_t buffer[9] = { m1 > 0, m2 > 0, m3 > 0, m4 > 0, (uint8_t)(fabs(m1) * 254.0f), (uint8_t)(fabs(m2) * 254.0f), (uint8_t)(fabs(m3) * 254.0f), (uint8_t)(fabs(m4) * 254.0f), 255 };

    // Send buffer
    write(buffer);
}
} // Robots
} // BoBRobotics
#endif	// __linux__
