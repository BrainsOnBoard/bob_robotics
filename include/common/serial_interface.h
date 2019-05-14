#pragma once

// BoB robotics include
#include "common/logging.h"

// Posix includes
#include <fcntl.h>
#include <unistd.h>

// Standard C includes
#include <cstdint>

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
    bool setup(const char *path);
    bool setAttributes(int speed);
    bool setBlocking(bool should_block);
    bool readByte(uint8_t &byte);

    template<typename T, size_t N>
    bool read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_Serial_fd, &data[0], size) != size) {
            LOGE << "Failed to read from serial port";
            return false;
        }
        else {
            return true;
        }
    }

    bool writeByte(uint8_t byte);

    // writes data
    template<typename T, size_t N>
    bool write(const T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::write(m_Serial_fd, &data[0], size) != size) {
            LOGE << "Failed to write to serial port";
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
    int m_Serial_fd;
};
} // BoBRobotics
