#include "common/bn055_imu.h"


#include <iostream>

//----------------------------------------------------------------------------
// BoBRobotics::BN055
//----------------------------------------------------------------------------
BoBRobotics::BN055::BN055(const char *path, int slaveAddress)
{
    init(path, slaveAddress);
}
//----------------------------------------------------------------------------
void BoBRobotics::BN055::init(const char *path, int slaveAddress)
{
    // Setup I2C device
    m_IMU.setup(path, slaveAddress);
    
    const uint8_t chipID = readByte(Register::CHIP_ID_ADDR);
    
    std::cout << chipID << std::endl;
}
//----------------------------------------------------------------------------
uint8_t BoBRobotics::BN055::readByte(uint8_t address)
{
    m_IMU.writeByte(address);
    return m_IMU.readByte();
}
//----------------------------------------------------------------------------
uint8_t BoBRobotics::BN055::readByte(Register reg)
{
    return readByte(static_cast<uint8_t>(reg));
}