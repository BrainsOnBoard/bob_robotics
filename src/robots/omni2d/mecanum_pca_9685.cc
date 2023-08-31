#ifdef __linux__
// BoB robotics includes
#include "common/macros.h"
#include "robots/omni2d/mecanum_pca_9685.h"

// Standard C includes
#include <cmath>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <vector>


namespace BoBRobotics {
namespace Robots {
namespace Omni2D {

const PCA9685::Channel MecanumPCA9685::motorChannels[MOTOR_MAX][MOTOR_CHANNEL_MAX] = {
        {PCA9685::Channel::CHANNEL_0, PCA9685::Channel::CHANNEL_1},
        {PCA9685::Channel::CHANNEL_2, PCA9685::Channel::CHANNEL_3},
        {PCA9685::Channel::CHANNEL_4, PCA9685::Channel::CHANNEL_5},
        {PCA9685::Channel::CHANNEL_6, PCA9685::Channel::CHANNEL_7}};
//----------------------------------------------------------------------------
MecanumPCA9685::MecanumPCA9685(int slaveAddress, const char *path)
  : m_PCA9685(slaveAddress, path)
{
}
//----------------------------------------------------------------------------
MecanumPCA9685::~MecanumPCA9685()
{
    stopMoving();
}
//----------------------------------------------------------------------------
// Omni2D virtuals
//----------------------------------------------------------------------------
void MecanumPCA9685::driveMotors(float m1, float m2, float m3, float m4)
{
    setMotorThrottle(Motor::MOTOR_1, m1);
    setMotorThrottle(Motor::MOTOR_2, m2);
    setMotorThrottle(Motor::MOTOR_3, m3);
    setMotorThrottle(Motor::MOTOR_4, m4);
}
//----------------------------------------------------------------------------
void MecanumPCA9685::setMotorThrottle(Motor motor, float throttle)
{
    BOB_ASSERT(motor >= MOTOR_1 && motor < MOTOR_MAX);
    BOB_ASSERT(throttle >= -1.0f && throttle <= 1.0f);

    // If throttle is zero, brake motor by turning on both channels
    if(throttle == 0.0f) {
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_POSITIVE], 1.0f);
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_NEGATIVE], 1.0f);
    }
    // Otherwise, if throttle is negative, turn off positive channel and turn on negative channel
    else if(throttle < 0.0f) {
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_POSITIVE], 0.0f);
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_NEGATIVE], -throttle);
    }
    // Otherwise, if throttle is positive, turn on negative channel and turn off positive
    else {
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_POSITIVE], throttle);
        m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_NEGATIVE], 0.0f);
    }
}
//----------------------------------------------------------------------------
void
MecanumPCA9685::omni2DInternal(float forward, float sideways, float turn)
{
    BOB_ASSERT(forward >= -1.0f && forward <= 1.0f);
    BOB_ASSERT(sideways >= -1.0f && sideways <= 1.0f);
    BOB_ASSERT(turn >= -1.0f && turn <= 1.0f);

    // resolve to motor speeds
    const float m1 = -sideways + forward - turn;
    const float m2 = +sideways + forward + turn;
    const float m3 = +sideways + forward - turn;
    const float m4 = -sideways + forward + turn;

    // Cap before passing to driveMotors as valid forward, sideways and
    // turn values can still result in invalid m1, m2, m3 and m4
    const auto cap = [](float val) { return std::min(1.0f, std::max(val, -1.0f)); };
    driveMotors(cap(m1), cap(m2), cap(m3), cap(m4));
}
} // Omni2D
} // Robots
} // BoBRobotics
#endif	// __linux__
