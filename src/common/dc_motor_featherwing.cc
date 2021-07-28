#ifdef __linux__
// BoB robotics includes
#include "dc_motor_featherwing.h"

//----------------------------------------------------------------------------
// BoBRobotics::Robots::DCMotorFeatherWing
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace Robots {
const PCA9685::Channel DCMotorFeatherWing::motorChannels[MOTOR_MAX][MOTOR_CHANNEL_MAX] = {
        {PCA9685::Channel::CHANNEL_8, PCA9685::Channel::CHANNEL_9, PCA9685::Channel::CHANNEL_10},
        {PCA9685::Channel::CHANNEL_13, PCA9685::Channel::CHANNEL_11, PCA9685::Channel::CHANNEL_12},
        {PCA9685::Channel::CHANNEL_2, PCA9685::Channel::CHANNEL_3, PCA9685::Channel::CHANNEL_4},
        {PCA9685::Channel::CHANNEL_7, PCA9685::Channel::CHANNEL_5, PCA9685::Channel::CHANNEL_6}};
//----------------------------------------------------------------------------
DCMotorFeatherWing::DCMotorFeatherWing(const char *path, int slaveAddress, units::frequency::hertz_t pwmFrequency)
:   m_PCA9685(path, slaveAddress)
{
    // Set PWM frequency
    m_PCA9685.setFrequency(pwmFrequency);
}
//----------------------------------------------------------------------------
void DCMotorFeatherWing::enableMotor(Motor motor)
{
    m_PCA9685.setDutyCycle(motorChannels[motor][MOTOR_CHANNEL_ENABLE], 1.0f);
}
//----------------------------------------------------------------------------
void DCMotorFeatherWing::setDCMotorThrottle(Motor motor, float throttle)
{
    // Clamp throttle value
    throttle = std::min(1.0f, std::max(-1.0f, throttle));
    
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

}   // namespace Robots
}   // namespace BoBRobotics
#endif   // __linux__