#pragma once

// Standard C++ includes
#include <iostream>
#include <vector>

// Standard C includes
#include <cstring>

// Posix includes
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <termios.h>
#include <netinet/in.h>

#include <stdint.h>
#include <stdbool.h>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// SerialInterface
//----------------------------------------------------------------------------
class SerialInterface
{
public:
    SerialInterface()
    {
    }

    SerialInterface(const char *path)
    {
        if(!setup(path)) {
            throw std::runtime_error("Cannot open Serial interface");
        }
    }

    ~SerialInterface()
    {
        // Close Serial port
        if(m_Serial_fd >= 0) {
            close(m_Serial_fd);
        }
    }

    //---------------------------------------------------------------------
    // Public API
    //---------------------------------------------------------------------
    bool setup(const char *path )
    {

		m_Serial_fd = open (path, O_WRONLY | O_NOCTTY | O_SYNC);
		if (m_Serial_fd < 0)
		{
		    std::cerr << "Error in setup:" << strerror(errno) << std::endl;
		    return false;
	    // set speed to 115,200 bps, 8n1 (no parity)
		} else if (setAttributes (m_Serial_fd, B9600)) {

			// set no blocking
			if (setBlocking (m_Serial_fd, 1)) {
				std::cout << "Serial successfully initialized" << std::endl;
				return true;
			}
		}
    	return false;

    }

    bool setAttributes (int m_Serial_fd, int speed)
	{

	    struct termios tty;
	    memset (&tty, 0, sizeof tty);
	    if (tcgetattr (m_Serial_fd, &tty) != 0)
	    {
	    	std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
	            return false;
	    }

	    cfsetospeed (&tty, speed);
	    cfsetispeed (&tty, speed);

				tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
			tty.c_cflag &= ~CSIZE;
			tty.c_cflag |= CS8;         /* 8-bit characters */
			tty.c_cflag &= ~PARENB;     /* no parity bit */
			tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
			tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

			/* setup for non-canonical mode */
			tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
			tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
			tty.c_oflag &= ~OPOST;

			/* fetch bytes as they become available */
			tty.c_cc[VMIN] = 1;
			tty.c_cc[VTIME] = 5;

	    if (tcsetattr (m_Serial_fd, TCSANOW, &tty) != 0)
	    {
	    	std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
	        return false;
	    }
	    return true;
	}

	bool setBlocking (int m_Serial_fd, int should_block)
	{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (m_Serial_fd, &tty) != 0)
        {
                std::cerr << "Error in setup from tcgetattr:" << strerror(errno) << std::endl;
		            return false;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (m_Serial_fd, TCSANOW, &tty) != 0) {
                std::cerr << "Error in setup setting term attributes:" << strerror(errno) << std::endl;
		        return false;
        }
        return true;
	}

    bool readByte(uint8_t &byte)
    {
        char data;// = i2c_smbus_read_byte(m_I2C);
        ::read (m_Serial_fd, &data, 1);
        if(data != 1) {
            std::cerr << "Failed to read byte from Serial port" << std::endl;
            return false;
        }
        else {
            byte = (uint8_t)data;
            return true;
        }
    }

    template<typename T, size_t N>
    bool read(T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::read(m_Serial_fd, &data[0], size) != size) {
            std::cerr << "Failed to read from Serial port" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    /*bool writeByteCommand(uint8_t address, uint8_t byte)
    {
        if(i2c_smbus_write_byte_data(m_I2C, address, byte) < 0) {
            std::cerr << "Failed to write byte to i2c bus" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }*/

    bool writeByte(uint8_t byte)
    {
        if(::write(m_Serial_fd, &byte, 1) < 0) {
            std::cerr << "Failed to write byte to Serial port" << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    // writes data
    template<typename T, size_t N>
    bool write(const T (&data)[N])
    {
        const size_t size = sizeof(T) * N;
        if (::write(m_Serial_fd, &data[0], size) != size) {
            std::cerr << "Failed to write to Serial port" << std::endl;
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
    int m_Serial_fd;                                      // i2c file
};
} // BoBRobotics
