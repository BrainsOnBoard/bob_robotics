#pragma once

// Posix includes
#include <termios.h>
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
    SerialInterface(const char *path);
    ~SerialInterface();

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    termios getAttributes() const;
    void setAttributes(const termios &tty) const;

    bool readByte(uint8_t &byte) const;

    template<typename T, size_t N>
    bool read(T (&data)[N]) const
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

    void writeByte(uint8_t byte) const;

    // writes data
    template<typename T, size_t N>
    void write(const T (&data)[N]) const
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
