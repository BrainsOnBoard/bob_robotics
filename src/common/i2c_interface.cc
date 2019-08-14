#if defined(__linux__) && !defined(NO_I2C)

// BoB robotics includes
#include "common/i2c_interface.h"
#include "common/logging.h"

// Standard C++ includes
#include <string>
#include <vector>

// Standard C includes
#include <cstring>

// POSIX includes
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C"
{
// I2C includes
#include <linux/i2c-dev.h>

// This extra header is needed after Ubuntu 16.04 (newer kernel?)
#ifndef I2C_SMBUS_BYTE_DATA
#include <i2c/smbus.h>
#endif
}

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::I2CInterface
//----------------------------------------------------------------------------
//! Class for communicating over I2C
I2CInterface::I2CInterface()
  : m_I2C(0)
{}

I2CInterface::I2CInterface(const char *path, int slaveAddress)
  : m_I2C(0)
{
    setup(path, slaveAddress);
}

I2CInterface::~I2CInterface()
{
    // Close I2C device
    if (m_I2C >= 0) {
        close(m_I2C);
    }

    LOG_DEBUG << "I2C closed";
}

//---------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------
void
I2CInterface::setup(const char *path, int slaveAddress)
{
    m_I2C = open(path, O_RDWR);
    if (m_I2C < 0) {
        throw std::runtime_error("Error in setup: " + std::string(strerror(errno)) + "\n" +
                                 "The error is usually permission error which, on Ubuntu, can be fixed by" +
                                 "creating a file /etc/udev/rules.d/90-i2c.rules and adding the following line:\n" +
                                 "   KERNEL==\"i2c-[0-7]\",MODE=\"0666\"");
    }

    if (ioctl(m_I2C, I2C_SLAVE, slaveAddress) < 0) {
        throw std::runtime_error("Cannot connect to I2C slave");
    } else {
        LOG_INFO << "I2C successfully initialized";
    }
}

uint8_t
I2CInterface::readByteCommand(uint8_t address)
{
    const auto data = i2c_smbus_read_byte_data(m_I2C, address);
    if (data < 0) {
        throw std::runtime_error("Failed to read byte from i2c bus");
    } else {
        return static_cast<uint8_t>(data);
    }
}

uint8_t
I2CInterface::readByte()
{
    const auto data = i2c_smbus_read_byte(m_I2C);
    if (data < 0) {
        throw std::runtime_error("Failed to read byte from i2c bus");
    } else {
        return static_cast<uint8_t>(data);
    }
}

void
I2CInterface::writeByteCommand(uint8_t address, uint8_t byte)
{
    if (i2c_smbus_write_byte_data(m_I2C, address, byte) < 0) {
        throw std::runtime_error("Failed to write byte to i2c bus");
    }
}

void
I2CInterface::writeByte(uint8_t byte)
{
    if (i2c_smbus_write_byte(m_I2C, byte) < 0) {
        throw std::runtime_error("Failed to write byte to i2c bus");
    }
}
} // BoBRobotics
#endif // __linux__ && !NO_I2C
