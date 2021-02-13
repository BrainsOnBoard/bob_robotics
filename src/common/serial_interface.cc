#ifdef __linux__ // This code is Linux only

// BoB robotics includes
#include "common/serial_interface.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <iostream>
#include <vector>

// Standard C includes
#include <cstring>

// Posix includes
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

namespace BoBRobotics {
SerialInterface::SerialInterface(const char *path)
  : m_Serial_fd{ open(path, O_WRONLY | O_NOCTTY | O_SYNC) }
{
    if (m_Serial_fd < 0) {
        throw std::runtime_error("Could not open serial interface: " + std::string(strerror(errno)));
    }

    LOGI << "Serial successfully initialised";
}

SerialInterface::~SerialInterface()
{
    // Close Serial port
    if (m_Serial_fd >= 0) {
        close(m_Serial_fd);
    }
}

//---------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------
termios
SerialInterface::getAttributes() const
{
    termios tty;
    if (tcgetattr(m_Serial_fd, &tty) != 0) {
        throw std::runtime_error("Error in call to tcgetattr: " + std::string(strerror(errno)));
    }

    return tty;
}

void
SerialInterface::setAttributes(const termios &tty) const
{
    if (tcsetattr(m_Serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error in call to tcsetattr:" + std::string(strerror(errno)));
    }
}

bool
SerialInterface::readByte(uint8_t &byte)
{
    const ssize_t ret = ::read(m_Serial_fd, reinterpret_cast<char *>(&byte), 1);
    if (ret < 0) {
        // No data on non-blocking socket
        if (errno == EAGAIN) {
            return false;
        }

        // Otherwise it's a proper error
        throw std::runtime_error("Failed to read byte from serial port");
    }

    return true;
}

void
SerialInterface::writeByte(uint8_t byte)
{
    if (::write(m_Serial_fd, &byte, 1) < 0) {
        throw std::runtime_error("Failed to write byte to serial port");
    }
}
} // BoBRobotics
#endif // __linux__
