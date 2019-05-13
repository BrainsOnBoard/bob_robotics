// BoB robotics includes
#include "common/circstat.h"
#include "common/logging.h"
#include "robots/ev3/ev3.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <utility>

using namespace units::length;
using namespace units::velocity;
using namespace units::literals;

namespace BoBRobotics {
namespace Robots {
void
checkMotor(const EV3::MotorType &motor, const std::string &label)
{
    const auto states = motor.state();
    for (auto &state : states) {
        if (state == "overloaded" || state == "stalled") {
            LOGW << label << " motor is " << state;
        }
    }
}

EV3::EV3(const ev3dev::address_type leftMotorPort,
         const ev3dev::address_type rightMotorPort)
  : m_MotorLeft(leftMotorPort)
  , m_MotorRight(rightMotorPort)
  , m_MaxSpeedTachos(m_MotorLeft.max_speed())
  , m_TachoCountPerRotation(m_MotorLeft.count_per_rot())
{}

EV3::~EV3()
{
    stopMoving();
    stopReadingFromNetwork();
}

void
EV3::stopMoving()
{
    m_MotorLeft.stop();
    m_MotorRight.stop();
    setWheelSpeeds(0.f, 0.f);
}

void
EV3::tank(float left, float right)
{
    setWheelSpeeds(left, right);

    const float maxTachos = getMaximumSpeedProportion() * m_MaxSpeedTachos;
    m_MotorLeft.set_speed_sp(maxTachos * left);
    m_MotorRight.set_speed_sp(maxTachos * right);
    m_MotorLeft.run_forever();
    m_MotorRight.run_forever();

    // Every 3s check if the motors are overloaded
    using namespace std::literals;
    if (m_MotorStatusTimer.elapsed() > 3s) {
        // Restart timer
        m_MotorStatusTimer.start();

        checkMotor(m_MotorLeft, "left");
        checkMotor(m_MotorRight, "right");
    }
}

millimeter_t
EV3::getRobotWidth() const
{
    return 12_cm;
}

meters_per_second_t
EV3::getAbsoluteMaximumSpeed() const
{
    return tachoToSpeed(m_MaxSpeedTachos);
}

std::pair<meters_per_second_t, meters_per_second_t>
EV3::getWheelVelocities() const
{
    return std::make_pair(tachoToSpeed(m_MotorLeft.speed()),
                          tachoToSpeed(m_MotorRight.speed()));
}

meters_per_second_t
EV3::tachoToSpeed(int tachos) const
{
    constexpr units::length::meter_t wheelRadius = 5.5_cm / 2;
    const double angularVelocity{ 2.0 * pi() * static_cast<double>(tachos) / static_cast<double>(m_TachoCountPerRotation) };
    return meters_per_second_t{ angularVelocity * wheelRadius.value() };
}
}
}