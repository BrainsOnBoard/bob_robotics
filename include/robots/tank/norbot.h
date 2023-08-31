#pragma once
#ifdef __linux__

// BoB robotics includes
#include "common/i2c_interface.h"
#include "robots/tank/tank_base.h"

// Third-party includes
#include "third_party/units.h"

namespace BoBRobotics {
namespace Robots {
namespace Tank {
//----------------------------------------------------------------------------
// BoBRobotics::Robots::Tank::Norbot
//----------------------------------------------------------------------------
//! An interface for wheeled, Arduino-based robots developed at the University of Sussex
class Norbot : public TankBase<Norbot>
{
    friend TankBase<Norbot>;
public:
    Norbot(const char *path = I2C_DEVICE_DEFAULT, int slaveAddress = 0x29);

    //----------------------------------------------------------------------------
    // TankBase virtuals
    //----------------------------------------------------------------------------
    ~Norbot();

    static constexpr auto getMaximumSpeed()
    {
        return units::velocity::meters_per_second_t{ 0.11 };
    }

    static constexpr auto getRobotWidth()
    {
        return units::length::millimeter_t{ 104 };
    }

private:
    I2CInterface m_I2C;

    void tankInternal(float left, float right);

    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        m_I2C.read(data);
    }

    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        m_I2C.write(data);
    }
}; // Norbot
} // Tank
} // Robots
} // BoBRobotics
#endif // __linux__
