#ifdef __linux__
#pragma once

// BoB robotics includes
#include "common/i2c_interface.h"
#include "robots/ackermann/ackermann_base.h"
#include "robots/ackermann/rc_car_common.h"

// Third-party includes
#include "third_party/units.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Robots {
namespace Ackermann {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Ackermann::PassiveRCCarBot
//----------------------------------------------------------------------------
//! An interface for RC car robots which are being passively driven by the
//! remote control.
class PassiveRCCarBot
{
    friend class RCCarBot;
    using degree_t = units::angle::degree_t;

public:
    PassiveRCCarBot(const char *path = I2C_DEVICE_DEFAULT);

    //! Read speed and turn values from remote control
    std::pair<float, degree_t> readRemoteControl();

    constexpr static degree_t TurnMax{ 35 };

private:
    BoBRobotics::I2CInterface m_I2C; // i2c interface

    PassiveRCCarBot(const char *path, RCCar::State initialState);
    void setState(RCCar::State state);
}; // PassiveRCCarBot

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Ackermann::RCCarBot
//----------------------------------------------------------------------------
//! An interface for 4 wheeled, Arduino-based robots developed at the University of Sussex
class RCCarBot final
  : public AckermannBase<RCCarBot>
{
    using degree_t = units::angle::degree_t;

public:
    RCCarBot(const char *path = I2C_DEVICE_DEFAULT);
    ~RCCarBot();

    std::pair<float, degree_t> readRemoteControl();

    void moveForward(float speed);
    constexpr static degree_t getMaximumTurn();

    void move(float speed, float left);

    //! Move the car with Speed: [-1,1], TurningAngle: [-35,35]
    void move(float speed, degree_t left);

    //! Stop the car
    void stopMoving();

private:
    PassiveRCCarBot m_Bot;
}; // RCCarBot
} // Ackermann
} // Robots
} // BoBRobotics
#endif	// __linux__
