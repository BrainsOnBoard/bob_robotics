#ifdef __linux__
#include "common/pca_9685.h"

// BoB robotics include
#include "plog/Log.h"

namespace
{
union PWM
{
    uint16_t ushort[2];
    uint8_t byte[4];
};
}
//----------------------------------------------------------------------------
// BoBRobotics::PCA9685
//----------------------------------------------------------------------------
namespace BoBRobotics
{
PCA9685::PCA9685(const char *path, int slaveAddress, units::frequency::hertz_t referenceClockSpeed)
:   m_ReferenceClockSpeed(referenceClockSpeed)
{
    // Setup I2C device
    m_Device.setup(path, slaveAddress);
    
    reset();
}
//----------------------------------------------------------------------------
void PCA9685::setDutyCycle(Channel channel, float dutyCycle)
{
    // Clamp duty cycle
    dutyCycle = std::min(1.0f, std::max(0.0f, dutyCycle));

    // If duty cycle is fully on, set full-on bit
    PWM pwm;
    if(dutyCycle == 1.0f) {
        pwm.ushort[0] = 0x1000;
        pwm.ushort[1] = 0;
    }
    // Otherwise, if it's fully off, set full-off bit
    else if(dutyCycle == 0.0f) {
        pwm.ushort[0] = 0;
        pwm.ushort[1] = 0x1000;;
    }
    // Otherwise, calculate off duty cycle
    else {
        pwm.ushort[0] = 0;
        pwm.ushort[1] = (uint16_t)std::round(dutyCycle * 4095.0f);
    }
    
    // Write the four bytes representing channel state to device
    const uint8_t startChannel = static_cast<uint8_t>(Register::LED0_ON_L) + (4 * static_cast<uint8_t>(channel));
    for(size_t i = 0; i < 4; i++) {
        writeByte(startChannel + i, pwm.byte[i]);
    }
}
//----------------------------------------------------------------------------
float PCA9685::getDutyCycle(Channel channel)
{    
    // Read 4 bytes representing state of each channel into union
    // **TOOD** investigate auto-increment
    PWM pwm;
    const uint8_t startChannel = static_cast<uint8_t>(Register::LED0_ON_L) + (4 * static_cast<uint8_t>(channel));
    for(size_t i = 0; i < 4; i++) {
        pwm.byte[i] = readByte(startChannel + i);
    }
    
    
    LOGI << pwm.ushort[0] << "," << pwm.ushort[1];
    // If full-on bit is set, return 1.0
    if(pwm.ushort[0] == 0x1000) {
        return 1.0f;
    }
    // Otherwise, if full-off bit is set, return 0.0
    else if(pwm.ushort[1] == 0x1000) {
        return 0.0f;
    }
    // Otherwise scale off into duty cycle
    else {
        return (float)pwm.ushort[1] / 4095.0f;
    }
}
//----------------------------------------------------------------------------
void PCA9685::setFrequency(units::frequency::hertz_t frequency)
{
    // Calculate prescale factor
    const double prescale = m_ReferenceClockSpeed / 4096.0 / frequency + 0.5;
    if(prescale < 3.0) {
        throw std::runtime_error("PCA9685 cannot output at the given frequency");
    }
    
    // Put device to sleep, turning of oscillator
    const uint8_t oldMode1 = readRegister(Register::MODE_1);
    writeRegister(Register::MODE_1, oldMode1 | mode1Sleep);
    
    // Set prescale
    writeRegister(Register::PRESCALE, (uint8_t)std::round(prescale));
    
    // Re-awaken device
    writeRegister(Register::MODE_1, oldMode1);
}
//----------------------------------------------------------------------------
units::frequency::hertz_t PCA9685::getFrequency()
{
    // Read pre-scale
    const uint8_t prescale = readRegister(Register::PRESCALE);
    
    // Convert to hertz and return
    return m_ReferenceClockSpeed / 4096.0 / prescale;
}
//----------------------------------------------------------------------------
void PCA9685::reset()
{
    writeByte(static_cast<uint8_t>(Register::MODE_1), 0);
}
//----------------------------------------------------------------------------
uint8_t PCA9685::readByte(uint8_t address)
{
    m_Device.writeByte(address);
    return m_Device.readByte();
}
//----------------------------------------------------------------------------
void PCA9685::writeByte(uint8_t address, uint8_t data)
{
    m_Device.writeByteCommand(address, data);
}
//----------------------------------------------------------------------------
uint8_t PCA9685::readRegister(Register reg)
{
    return readByte(static_cast<uint8_t>(reg));
}
//----------------------------------------------------------------------------
void PCA9685::writeRegister(Register reg, uint8_t data)
{
    writeByte(static_cast<uint8_t>(reg), data);
}
}   // BoBRobotics
#endif  // __linux__