#include "common/bn055_imu.h"

// Standard C++ includes
#include <chrono>
#include <iostream>
#include <thread>

// BoB robotics include
#include "common/logging.h"

//----------------------------------------------------------------------------
// BoBRobotics::BN055
//----------------------------------------------------------------------------
namespace BoBRobotics
{
constexpr uint8_t BN055::imuID;
//----------------------------------------------------------------------------
BN055::BN055(OperationMode mode, const char *path, int slaveAddress)
{
    init(mode, path, slaveAddress);
}
//----------------------------------------------------------------------------
void BN055::init(OperationMode mode, const char *path, int slaveAddress)
{
    using namespace std::chrono_literals;
    
    // Setup I2C device
    m_IMU.setup(path, slaveAddress);
    
    // Read chip ID and check it matches
    const uint8_t chipID = readRegister(Register::CHIP_ID_ADDR);
    if(chipID != imuID) {
        throw std::runtime_error("Incorrect chip id at slave address:" + std::to_string(slaveAddress));
    }
    
    LOGI << "Switching mode";
    // Switch to config mode (just in case since this is the default)
    setMode(OperationMode::CONFIG);
  
    LOGI << "Resetting";
    
    // Reset
    writeRegister(Register::SYS_TRIGGER_ADDR, 0x20);
    std::this_thread::sleep_for(1000ms);

    LOGI << "Waiting";
    // Wait until chip comes back up
    while(readRegister(Register::CHIP_ID_ADDR) != imuID) {
        std::this_thread::sleep_for(10ms);
    }
    
    LOGI << "Setting power mode";
    // Set to normal power mode
    writeRegister(Register::PWR_MODE_ADDR, static_cast<uint8_t>(PowerMode::NORMAL));
    
    LOGI << "Setting mode";
    writeRegister(Register::PAGE_ID_ADDR, 0);
    writeRegister(Register::SYS_TRIGGER_ADDR, 0);
    setMode(mode);
}
//----------------------------------------------------------------------------
Eigen::Vector3f BN055::getVector(VectorType vectorType)
{
    // Read vector from register
    int16_t raw[3];
    readData(static_cast<uint8_t>(vectorType), raw);
    
    if(vectorType == VectorType::MAGNETOMETER
        || vectorType == VectorType::GYROSCOPE
        || vectorType == VectorType::EULER)
    {
        return Eigen::Vector3f((float)raw[0] / 16.0f, (float)raw[1] / 16.0f, (float)raw[2] / 16.0f);
    }
    else {
        return Eigen::Vector3f((float)raw[0] / 100.0f, (float)raw[1] / 100.0f, (float)raw[2] / 100.0f);
    }
    
}
//----------------------------------------------------------------------------
uint8_t BN055::readByte(uint8_t address)
{
    m_IMU.writeByte(address);
    return m_IMU.readByte();
}
//----------------------------------------------------------------------------
void BN055::writeByte(uint8_t address, uint8_t data)
{
    m_IMU.writeByteCommand(address, data);
}
//----------------------------------------------------------------------------
uint8_t BN055::readRegister(Register reg)
{
    return readByte(static_cast<uint8_t>(reg));
}
//----------------------------------------------------------------------------
void BN055::writeRegister(Register reg, uint8_t data)
{
    writeByte(static_cast<uint8_t>(reg), data);
}
//----------------------------------------------------------------------------
BN055::OperationMode BN055::setMode(OperationMode mode)
{
    // Stash old mode
    const OperationMode oldMode = m_Mode;
    
    // Store new mode and write register
    m_Mode = mode;
    writeRegister(Register::OPR_MODE_ADDR, static_cast<uint8_t>(mode));
    
    return oldMode;
}
}   // namespace BoBRobotics