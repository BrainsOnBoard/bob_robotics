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
Mecanum::Mecanum(const char *path)
  : m_Serial(path)
  , m_Forward(0.0f)
  , m_Sideways(0.0f)
  , m_Turn(0.0f)
{}

//----------------------------------------------------------------------------
// Omni2D virtuals
//----------------------------------------------------------------------------
void
Mecanum::omni2D(float forwards, float sideways, float turn)
{
    // Cache left and right
    m_Forward = forwards;
    m_Sideways = sideways;
    m_Turn = turn;

    // resolve to motor speeds
    float m1 = (m_Sideways + m_Forward - m_Turn);
    float m2 = (-m_Sideways + m_Forward + m_Turn);
    float m3 = (-m_Sideways - m_Forward - m_Turn);
    float m4 = (+m_Sideways - m_Forward + m_Turn);

    // clamp values to be between -1 and 1 after resolving
    const auto cap = [](float &val) {
        val = std::min(1.f, std::max(val, -1.f));
    };
    cap(m1);
    cap(m2);
    cap(m3);
    cap(m4);

    // **NOTE** 254s are because 255 is line end character
    uint8_t buffer[9] = { m1 > 0, m2 > 0, m3 > 0, m4 > 0, (uint8_t)(fabs(m1) * 254.0f), (uint8_t)(fabs(m2) * 254.0f), (uint8_t)(fabs(m3) * 254.0f), (uint8_t)(fabs(m4) * 254.0f), 255 };

    // Send buffer
    write(buffer);
}

float
Mecanum::getForwards() const
{
    return m_Forward;
}

float
Mecanum::getSideways() const
{
    return m_Sideways;
}

float
Mecanum::getTurn() const
{
    return m_Turn;
}
} // Robots
} // BoBRobotics
