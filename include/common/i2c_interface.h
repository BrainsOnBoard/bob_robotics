#pragma once
#ifdef __linux__

// POSIX includes
#include <unistd.h>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <stdexcept>

// If we're on Jetson TX1, I2C bus 1 is the one broken out
#if TEGRA_CHIP_ID == 33
    #define I2C_DEVICE_DEFAULT "/dev/i2c-1"
// Whereas, on Jetson TX2, I2C bus 0 is the one broken out
#elif TEGRA_CHIP_ID == 24
    #define I2C_DEVICE_DEFAULT "/dev/i2c-0"
// Whereas, on Jetson Xavier NX with default settings, I2C bus 1 is broken out on pins 27 and 28
#elif TEGRA_CHIP_ID == 25
    #define I2C_DEVICE_DEFAULT "/dev/i2c-1"
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

    // Make non-moveable and non-copyable
    I2CInterface(I2CInterface &&) = delete;

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    void setup(const char *path, int slaveAddress);
    uint8_t readByteCommand(uint8_t address);
    uint8_t readByte();

    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        read(&data[0], N * sizeof(T));
    }

    template<typename T,
             typename std::enable_if<std::is_pod<T>::value, T>::type* = nullptr>
    void read(T &data)
    {
        read(&data, sizeof(T));
    }

    void writeByteCommand(uint8_t address, uint8_t byte);
    void writeByte(uint8_t byte);

    // writes data
    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        write(&data[0], N * sizeof(T));
    }

    template<typename T,
             typename std::enable_if<std::is_pod<T>::value, T>::type* = nullptr>
    void write(const T &data)
    {
        write(&data, sizeof(T));
    }

private:
    //---------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------
    int m_I2C; // i2c file

    void read(void *data, size_t size);
    void write(const void *data, size_t size);
};
} // BoBRobotics
#endif // __linux__
