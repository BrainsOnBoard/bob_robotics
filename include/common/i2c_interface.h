#pragma once
#ifdef __linux__

// POSIX includes
#include <unistd.h>

// Standard C++ includes
#include <stdexcept>

// If we're on Jetson TX1, I2C bus 1 is the one broken out
#if TEGRA_CHIP_ID == 33
    #define I2C_DEVICE_DEFAULT "/dev/i2c-1"
// Whereas, on Jetson TX2, I2C bus 0 is the one broken out
#elif TEGRA_CHIP_ID == 24
    #define I2C_DEVICE_DEFAULT "/dev/i2c-0"
// Otherwise, use /dev/null
#else
    #define I2C_DEVICE_DEFAULT "/dev/null"
#endif

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
