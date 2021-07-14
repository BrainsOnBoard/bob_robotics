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

constexpr degree_t RCCarBot::TurnMax;

RCCarBot::RCCarBot(const char *path)
  : m_I2C(path, RCCAR_SLAVE_ADDRESS)
  , m_speed(0.0f)
  , m_turningAngle(0_deg)
{
    setState(RCCar::State::I2CControl);
}

RCCarBot::~RCCarBot()
{
    stopMoving();
    setState(RCCar::State::RemoteControl);
}

//! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
void
RCCarBot::move(float speed, degree_t left)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);
    BOB_ASSERT(left >= -TurnMax && left <= TurnMax);

    RCCar::Message msg;
    msg.command = RCCar::Command::Drive;
    msg.move.speed = static_cast<int8_t>(speed * 100.f);
    msg.move.turn = static_cast<int8_t>(100.0 * left / TurnMax);
    m_I2C.write(&msg, sizeof(msg));
}

void
RCCarBot::moveForward(float speed)
{
    m_speed = speed;
    move(speed, m_turningAngle);
}

void
RCCarBot::steer(float left)
{
    steer(left * TurnMax);
}

void
RCCarBot::steer(units::angle::degree_t left)
{
    move(m_speed, left);
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
    return TurnMax;
}

void
RCCarBot::setState(RCCar::State state)
{
    RCCar::Message msg;
    msg.command = RCCar::Command::SetState;
    msg.state = state;
    m_I2C.write(&msg, sizeof(msg));
}


std::pair<float, degree_t>
RCCarBot::readRemoteControl()
{
    RCCar::Message msg;
    msg.command = RCCar::Command::ReadRemoteControl;
    m_I2C.write(&msg, sizeof(msg));

    RCCar::Movement move;
    m_I2C.read(&move, sizeof(move));

    return { float(move.speed) / 100.f, TurnMax * double(move.turn) / 100.0 };
}
} // Robots
} // BoBRobotics
#endif	// __linux__
