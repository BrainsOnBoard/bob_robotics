#ifdef __linux__ // This code is Linux only

// BoB robotics includes
#include "common/logging.h"
#include "common/serial_interface.h"

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
#include <termios.h>
#include <unistd.h>

namespace BoBRobotics {
SerialInterface::SerialInterface()
{
}

SerialInterface::SerialInterface(const char *path)
{
    setup(path);
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
void
SerialInterface::setup(const char *path)
{

    m_Serial_fd = open(path, O_WRONLY | O_NOCTTY | O_SYNC);
    if (m_Serial_fd < 0) {
        throw std::runtime_error("Could not open serial interface: " + std::string(strerror(errno)));
    }

    // set speed to 115,200 bps, 8n1 (no parity)
    setAttributes(B9600);

    // set no blocking
    setBlocking(true);

    LOGI << "Serial successfully initialised";
}

void
SerialInterface::setAttributes(int speed)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(m_Serial_fd, &tty) != 0) {
        throw std::runtime_error("Error in setup from tcgetattr: " + std::string(strerror(errno)));
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      /* 8-bit characters */
    tty.c_cflag &= ~PARENB;  /* no parity bit */
    tty.c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(m_Serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error in setup from tcgetattr:" + std::string(strerror(errno)));
    }
}

void
SerialInterface::setBlocking(bool should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(m_Serial_fd, &tty) != 0) {
        throw std::runtime_error("Error in setup from tcgetattr:" + std::string(strerror(errno)));
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr(m_Serial_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error("Error in setup setting term attributes:" + std::string(strerror(errno)));
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
