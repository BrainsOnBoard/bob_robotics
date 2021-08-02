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
class Norbot : public TankBase
{
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;

public:
    Norbot(const char *path = I2C_DEVICE_DEFAULT, int slaveAddress = 0x29);

    //----------------------------------------------------------------------------
    // TankBase virtuals
    //----------------------------------------------------------------------------
    virtual ~Norbot() override;

    virtual void tank(float left, float right) override;

    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override;

    virtual millimeter_t getRobotWidth() const override;

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
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

private:

    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    BoBRobotics::I2CInterface m_I2C;
}; // Norbot
} // Tank
} // Robots
} // BoBRobotics
#endif // __linux__
