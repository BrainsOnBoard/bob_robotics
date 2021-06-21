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
    bool readByte(uint8_t &byte);

    template<typename T, size_t N>
    bool read(T (&data)[N])
    {
        if (::read(m_Serial_fd, &data[0], sizeof(T) * N) < 0) {
            // Then we're waiting for data on a non-blocking socket
            if (errno == EAGAIN) {
                return false;
            }

            // Otherwise it's a proper error
            throw std::runtime_error("Failed to read from serial port");
        }

        return true;
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
