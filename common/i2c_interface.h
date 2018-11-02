#pragma once

// Standard C++ includes
#include <iostream>
#include <string>
#include <vector>

// Standard C includes
#include <cstring>

// POSIX includes
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>

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
class I2CInterface
{
public:
    I2CInterface() : m_I2C(0)
    {}

    I2CInterface(const char *path, int slaveAddress) : m_I2C(0)
    {
        setup(path, slaveAddress);
    }

    ~I2CInterface()
    {
        // Close I2C device
        if(m_I2C >= 0) {
            close(m_I2C);
        }
    }

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    void setup(const char *path, int slaveAddress )
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
            std::cout << "I2C successfully initialized" << std::endl;
        }
    }

    bool readByteCommand(uint8_t address, uint8_t &byte)
    {
        auto data = i2c_smbus_read_byte_data(m_I2C, address);
        if(data < 0) {
            std::cerr << "Failed to read byte from i2c bus" << std::endl;
            return false;
        }
        else {
            byte = (uint8_t)data;
            return true;
        }
    }

    bool readByte(uint8_t &byte)
    {
        auto data = i2c_smbus_read_byte(m_I2C);
        if(data < 0) {
            std::cerr << "Failed to read byte from i2c bus" << std::endl;
            return false;
        }
        else {
            byte = (uint8_t)data;
            return true;
        }
    }

    template<typename T, size_t N>
    bool read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_I2C, &data[0], size) != size) {
            std::cerr << "Failed to read from i2c bus" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    bool writeByteCommand(uint8_t address, uint8_t byte)
    {
        if(i2c_smbus_write_byte_data(m_I2C, address, byte) < 0) {
            std::cerr << "Failed to write byte to i2c bus" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    bool writeByte(uint8_t byte)
    {
        if(i2c_smbus_write_byte(m_I2C, byte) < 0) {
            std::cerr << "Failed to write byte to i2c bus" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    // writes data
    template<typename T, size_t N>
    bool write(const T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::write(m_I2C, &data[0], size) != size) {
            std::cerr << "Failed to write to i2c bus" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

private:
    //---------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------
    int m_I2C;                                      // i2c file
};
} // BoBRobotics
