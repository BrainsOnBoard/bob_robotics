#ifdef __linux__
// BoB robotics includes
#include "robots/rc_car_bot.h"
#include "common/macros.h"

using namespace units::angle;
using namespace units::literals;

// sign function
template<typename T>
int
sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

namespace BoBRobotics {
namespace Robots {

RCCarBot::RCCarBot(const char *path, int slaveAddress)
  : m_I2C(path, slaveAddress)
  , m_speed(0.0f)
  , m_turningAngle(0_deg)
{}

RCCarBot::~RCCarBot()
{
    stopMoving();
}

//! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
void
RCCarBot::move(float speed, degree_t turningAngle)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);
    BOB_ASSERT(turningAngle >= -35_deg && turningAngle <= 35_deg);

    m_speed = speed;
    m_turningAngle = turningAngle;

    // mapping to the range
    uint8_t uspeed = (speed * 255) + sgn(-speed) * 127;     // mapping to : 0-127 backward, 127-255 forward
    uint8_t uturn = (uint8_t)(90.0 + turningAngle.value()); // mapping to : 65 full left 90 center 125 full right
    uint8_t buffer[2] = { uspeed, uturn };
    m_I2C.write(buffer); // send to arduino on i2c
}

void
RCCarBot::moveForward(float speed)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);

    m_speed = speed;

    // mapping to the range
    uint8_t uspeed = (speed * 255) + sgn(-speed) * 127;     // mapping to : 0-127 backward, 127-255 forward
    uint8_t uturn = (uint8_t)(90.0 + m_turningAngle.value()); // mapping to : 65 full left 90 center 125 full right
    uint8_t buffer[2] = { uspeed, uturn };
    m_I2C.write(buffer); // send to arduino on i2c
}

void
RCCarBot::steer(float turningAngle)
{
    steer(turningAngle * 35_deg);
}

void
RCCarBot::steer(units::angle::degree_t turningAngle)
{
    BOB_ASSERT(turningAngle >= -35_deg && turningAngle <= 35_deg);

    m_turningAngle = turningAngle;

    // mapping to the range
    uint8_t uspeed = (m_speed * 255) + sgn(-m_speed) * 127;     // mapping to : 0-127 backward, 127-255 forward
    uint8_t uturn = (uint8_t)(90.0 + turningAngle.value()); // mapping to : 65 full left 90 center 125 full right
    uint8_t buffer[2] = { uspeed, uturn };
    m_I2C.write(buffer); // send to arduino on i2c
}

// stops the car
void
RCCarBot::stopMoving()
{
    move(0, 0_deg);
}

float
RCCarBot::getSpeed() const
{
    return m_speed;
}

degree_t
RCCarBot::getTurningAngle() const
{
    return m_turningAngle;
}

degree_t
RCCarBot::getMaximumTurn() const
{
    return 35_deg;
}

} // Robots
} // BoBRobotics
#endif	// __linux__