#pragma once
#ifdef __linux__

// POSIX includes
#include <unistd.h>

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::I2CInterface
//----------------------------------------------------------------------------
//! Class for communicating over I2C
class I2CInterface
{
public:
    I2CInterface();
    I2CInterface(const char *path, int slaveAddress);
    ~I2CInterface();

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    void setup(const char *path, int slaveAddress);
    uint8_t readByteCommand(uint8_t address);
    uint8_t readByte();

    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_I2C, &data[0], size) != size) {
            throw std::runtime_error("Failed to read from i2c bus");
        }
    }

    void writeByteCommand(uint8_t address, uint8_t byte);
    void writeByte(uint8_t byte);

    // writes data
    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::write(m_I2C, &data[0], size) != size) {
            throw std::runtime_error("Failed to write to i2c bus");
        }
    }

private:
    //---------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------
    int m_I2C; // i2c file
};
} // BoBRobotics
#endif // __linux__
