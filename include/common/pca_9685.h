#pragma once
#ifdef __linux__

// Standard C includes
#include <cstdint>

// BoB robotics third party includes
#include "third_party/units.h"

// BoB robotics includes
#include "i2c_interface.h"

//----------------------------------------------------------------------------
// BoBRobotics::PCA9685
//----------------------------------------------------------------------------
//! Userspace drive for PCA9685 pulse-width modulation (PWM) chip
//! Commonly used for controlling servos, LEDs and motors
namespace BoBRobotics {
class PCA9685
{
public:
    PCA9685(int slaveAddress, const char *path = I2C_DEVICE_DEFAULT,
            units::frequency::hertz_t referenceClockSpeed = units::frequency::megahertz_t(25.0));

    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    enum class Channel : uint8_t
    {
        CHANNEL_0 = 0,
        CHANNEL_1 = 1,
        CHANNEL_2 = 2,
        CHANNEL_3 = 3,
        CHANNEL_4 = 4,
        CHANNEL_5 = 5,
        CHANNEL_6 = 6,
        CHANNEL_7 = 7,
        CHANNEL_8 = 8,
        CHANNEL_9 = 9,
        CHANNEL_10 = 10,
        CHANNEL_11 = 11,
        CHANNEL_12 = 12,
        CHANNEL_13 = 13,
        CHANNEL_14 = 14,
        CHANNEL_15 = 15,
    };

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    //! Set the duty cycle of a channel from 0.0 (never high) to 1.0 (always high)
    void setDutyCycle(Channel channel, float dutyCycle);

    //! Get the duty cycle of a channel from 0.0 (never high) to 1.0 (always high)
    float getDutyCycle(Channel channel);

    //! Set PWM frequency
    void setFrequency(units::frequency::hertz_t frequency);

    //! Get PWM frequency
    units::frequency::hertz_t getFrequency();

    //! Reset device
    void reset();

private:
    static constexpr uint8_t mode1Sleep = (1 << 4);
    static constexpr uint8_t mode1AutoIncrement = (1 << 5);

    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    // Registers
    enum class Register : uint8_t
    {
        MODE_1 = 0x0,

        LED0_ON_L = 0x6,
        LED0_ON_H = 0x7,
        LED0_OFF_L = 0x8,
        LED0_OFF_H = 0x9,

        PRESCALE = 0xFE,
    };



    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    uint8_t readByte(uint8_t address);
    void writeByte(uint8_t address, uint8_t data);

    uint8_t readRegister(Register reg);
    void writeRegister(Register reg, uint8_t data);

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    const units::frequency::hertz_t m_ReferenceClockSpeed;
    I2CInterface m_Device;
};
}   // namespace BoBRobotics
#endif // __linux__
