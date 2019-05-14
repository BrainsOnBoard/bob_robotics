// BoB robotics includes
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
    if (!setup(path)) {
        throw std::runtime_error("Cannot open serial interface");
    }
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
bool
SerialInterface::setup(const char *path)
{

    m_Serial_fd = open(path, O_WRONLY | O_NOCTTY | O_SYNC);
    if (m_Serial_fd < 0) {
        LOGE << "Error in setup:" << strerror(errno);
        return false;
        // set speed to 115,200 bps, 8n1 (no parity)
    } else if (setAttributes(B9600)) {

        // set no blocking
        if (setBlocking(true)) {
            LOGI << "Serial successfully initialized";
            return true;
        }
    }
    return false;
}

bool
SerialInterface::setAttributes(int speed)
{

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(m_Serial_fd, &tty) != 0) {
        std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
        return false;
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
        std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool
SerialInterface::setBlocking(bool should_block)
{
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(m_Serial_fd, &tty) != 0) {
        std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
        return false;
    }

    tty.c_cc[VMIN] = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    if (tcsetattr(m_Serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error in setup setting term attributes:" << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool
SerialInterface::readByte(uint8_t &byte)
{
    char data; // = i2c_smbus_read_byte(m_I2C);
    ::read(m_Serial_fd, &data, 1);
    if (data != 1) {
        std::cerr << "Failed to read byte from Serial port" << std::endl;
        return false;
    } else {
        byte = (uint8_t) data;
        return true;
    }
}

bool
SerialInterface::writeByte(uint8_t byte)
{
    if (::write(m_Serial_fd, &byte, 1) < 0) {
        std::cerr << "Failed to write byte to Serial port" << std::endl;
        return false;
    } else {
        return true;
    }
}
} // BoBRobotics
