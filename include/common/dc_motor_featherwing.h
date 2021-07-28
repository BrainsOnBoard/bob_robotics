#pragma once
#ifdef __linux__

// BoB robotics includes
#include "pca_9685.h"

// Third-party includes
#include "third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::Robots::DCMotorFeatherWing
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace Robots {
class DCMotorFeatherWing
{
public:
    DCMotorFeatherWing(const char *path = I2C_DEVICE_DEFAULT, int slaveAddress = 0x60, 
                       units::frequency::hertz_t pwmFrequency = units::frequency::hertz_t(1600));
    
    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
    enum Motor
    {
        MOTOR_1,
        MOTOR_2,
        MOTOR_3,
        MOTOR_4,
        MOTOR_MAX,
    };
    
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void enableMotor(Motor motor);
    void setDCMotorThrottle(Motor motor, float throttle);
    
private:
    //------------------------------------------------------------------------
    // Enumerations
    //------------------------------------------------------------------------
    enum MotorChannel
    {
        MOTOR_CHANNEL_ENABLE,
        MOTOR_CHANNEL_POSITIVE,
        MOTOR_CHANNEL_NEGATIVE,
        MOTOR_CHANNEL_MAX
    };
    
    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    //! Mapping of motors to PCA9685 PWM channels
    static const PCA9685::Channel motorChannels[MOTOR_MAX][MOTOR_CHANNEL_MAX];
    
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    //! PCA9685 chip all the motors are connected to
    PCA9685 m_PCA9685;
};
}   // namespace Robots
}   // namespace BoBRobotics

#endif   // __linux__