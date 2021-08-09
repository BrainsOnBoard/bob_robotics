#ifdef __linux__
// BoB robotics includes
#include "common/macros.h"
#include "robots/ackermann/rc_car_bot.h"

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
namespace Ackermann {

constexpr degree_t PassiveRCCarBot::TurnMax;

PassiveRCCarBot::PassiveRCCarBot(const char *path)
  : PassiveRCCarBot(path, RCCar::State::RemoteControl)
{}

PassiveRCCarBot::PassiveRCCarBot(const char *path, RCCar::State initialState)
  : m_I2C(path, RCCAR_SLAVE_ADDRESS)
{
    setState(initialState);
}

void
PassiveRCCarBot::setState(RCCar::State state)
{
    RCCar::Message msg;

    // The SetState* commands align with the values of the State enum
    msg.command = static_cast<RCCar::Command>(state);

    m_I2C.write(msg);
}

std::pair<float, degree_t>
PassiveRCCarBot::readRemoteControl()
{
    RCCar::Message msg;
    msg.command = RCCar::Command::ReadRemoteControl;
    m_I2C.write(msg);

    RCCar::Movement move;
    m_I2C.read(move);

    return { float(move.speed) / 100.f, TurnMax * double(move.turn) / 100.0 };
}

RCCarBot::RCCarBot(const char *path)
  : m_Bot(path, RCCar::State::I2CControl)
{}

RCCarBot::~RCCarBot()
{
    stopMoving();
    m_Bot.setState(RCCar::State::RemoteControl);
}

//! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
void
RCCarBot::move(float speed, degree_t left)
{
    BOB_ASSERT(speed >= -1.f && speed <= 1.f);
    BOB_ASSERT(units::math::abs(left) <= PassiveRCCarBot::TurnMax);

    RCCar::Message msg;
    msg.command = RCCar::Command::Drive;
    msg.move.speed = static_cast<int8_t>(speed * 100.f);
    msg.move.turn = static_cast<int8_t>(100.0 * left / PassiveRCCarBot::TurnMax);
    m_Bot.m_I2C.write(msg);
}

void
RCCarBot::moveForward(float speed)
{
    move(speed, 0_deg);
}

// stops the car
void
RCCarBot::stopMoving()
{
    move(0, 0_deg);
}

std::pair<float, degree_t>
RCCarBot::readRemoteControl()
{
    return m_Bot.readRemoteControl();
}

degree_t
RCCarBot::getMaximumTurn()
{
    return PassiveRCCarBot::TurnMax;
}

} // Ackermann
} // Robots
} // BoBRobotics
#endif	// __linux__
