#ifdef __linux__
#pragma once

// BoB robotics includes
#include "common/pca_9685.h"
#include "omni2d.h"

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

namespace BoBRobotics {
namespace Robots {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::MecanumPCA9685
//----------------------------------------------------------------------------
class MecanumPCA9685 : public Omni2D
{
public:
    MecanumPCA9685(int slaveAddress = 0x40, const char *path = I2C_DEVICE_DEFAULT);
    virtual ~MecanumPCA9685();

    //----------------------------------------------------------------------------
    // Omni2D virtuals
    //----------------------------------------------------------------------------
    virtual void omni2D(float forward, float sideways, float turn) override;

    //! Drive robot using individual motor speeds - all range from -1 to 1
    void driveMotors(float m1, float m2, float m3, float m4);

private:
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
    
    enum MotorChannel
    {
        MOTOR_CHANNEL_POSITIVE,
        MOTOR_CHANNEL_NEGATIVE,
        MOTOR_CHANNEL_MAX
    };
    
    //------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------
    //! Mapping of motors to PCA9685 PWM channels
    static const PCA9685::Channel motorChannels[MOTOR_MAX][MOTOR_CHANNEL_MAX];
    
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void setMotorThrottle(Motor motor, float throttle);
    
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    PCA9685 m_PCA9685;
}; // Mecanum
} // Robots
} // BoBRobotics
#endif	// __linux__
