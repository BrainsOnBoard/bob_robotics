#pragma once

// Posix includes
#include <fcntl.h>
#include <unistd.h>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// SerialInterface
//----------------------------------------------------------------------------
class SerialInterface
{
public:
    SerialInterface();
    SerialInterface(const char *path);
    virtual ~SerialInterface();

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    void setup(const char *path);
    void setAttributes(int speed);
    void setBlocking(bool should_block);
    void readByte(uint8_t &byte);

    template<typename T, size_t N>
    void read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_Serial_fd, &data[0], size) != size) {
            throw std::runtime_error("Failed to read from serial port");
        }
    }

    void writeByte(uint8_t byte);

    // writes data
    template<typename T, size_t N>
    void write(const T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::write(m_Serial_fd, &data[0], size) != size) {
            throw std::runtime_error("Failed to write to serial port");
        }
    }

private:
    //---------------------------------------------------------------------
    // Members
    //---------------------------------------------------------------------
    int m_Serial_fd;
};
} // BoBRobotics
