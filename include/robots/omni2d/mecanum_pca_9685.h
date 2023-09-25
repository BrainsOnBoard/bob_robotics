#ifdef __linux__
#pragma once

// BoB robotics includes
#include "common/pca_9685.h"
#include "robots/omni2d/omni2d_base.h"

// Standard C++ includes
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdint>

namespace BoBRobotics {
namespace Robots {
namespace Omni2D {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Omni2D::MecanumPCA9685
//----------------------------------------------------------------------------
class MecanumPCA9685 : public Omni2DBase<MecanumPCA9685>
{
    friend Omni2DBase<MecanumPCA9685>;
public:
    MecanumPCA9685(int slaveAddress = 0x40, const char *path = I2C_DEVICE_DEFAULT);
    ~MecanumPCA9685();

    //! Drive robot using individual motor speeds - all range from -1 to 1
    void driveMotors(float m1, float m2, float m3, float m4);

protected:
    void omni2DInternal(float forward, float sideways, float turn);

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
} // Omni2D
} // Robots
} // BoBRobotics
#endif	// __linux__
