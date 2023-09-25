#ifdef __linux__
// BoB robotics includes
#include "robots/tank/norbot.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>

using namespace units::literals;
using namespace units::length;
using namespace units::velocity;

uint8_t floatToI2C(float speed)
{
    return (uint8_t) std::min(
            255.0f,
            std::max(0.0f, std::round(((speed + 1.0f) / 2.0f) * 255.0f)));
}

namespace BoBRobotics {
namespace Robots {
namespace Tank {

Norbot::Norbot(const char *path, int slaveAddress)
    : m_I2C(path, slaveAddress)
{
    // Sometimes Norbots get stuck driving, so let's stop it if we need to
    stopMoving();
}

//----------------------------------------------------------------------------
// TankBase virtuals
//----------------------------------------------------------------------------
Norbot::~Norbot()
{
    stopMoving();
}

void Norbot::tankInternal(float left, float right)
{
    // Convert standard (-1,1) values to bytes in order to send to I2C slave
    uint8_t buffer[2] = { floatToI2C(left), floatToI2C(right) };

    // Send buffer
    write(buffer);
}

} // Tank
} // Robots
} // BoBRobotics
#endif
