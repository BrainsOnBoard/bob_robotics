#pragma once

// BoB robotics includes
#include "tank.h"

// POSIX networking includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

// Standard C includes
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>


namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Surveyor
//----------------------------------------------------------------------------
/*!
 * \brief An interface for Active Robots' Surveyor line of robots
 *
 * See https://www.active-robots.com/surveyor-rover.html
 */
class Surveyor : public Tank
{
    using millimeter_t = units::length::millimeter_t;

public:
    Surveyor(const std::string &address, uint16_t port)
    {
        // Create socket
        m_Socket = socket(AF_INET, SOCK_STREAM, 0);
        if (m_Socket < 0) {
            throw std::runtime_error("Cannot open socket");
        }

        // Create socket address structure
        sockaddr_in destAddress;
        destAddress.sin_family = AF_INET;
        destAddress.sin_port = htons(port);
        destAddress.sin_addr.s_addr = inet_addr(address.c_str());

        // Connect socket
        if (connect(m_Socket,
                    reinterpret_cast<sockaddr *>(&destAddress),
                    sizeof(destAddress)) < 0) {
            throw std::runtime_error("Cannot connect socket to " + address +
                                     ":" + std::to_string(port));
        }
    }

    virtual ~Surveyor() override
    {
        stopMoving();
        stopReadingFromNetwork();

        if (m_Socket > 0) {
            close(m_Socket);
        }
    }

    //----------------------------------------------------------------------------
    // Tank virtuals
    //----------------------------------------------------------------------------
    virtual void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);

        // Scale and convert to int
        const float maxSpeed = 100.f * getMaximumSpeedProportion();
        int leftInt = (int) std::round(left * maxSpeed);
        int rightInt = (int) std::round(right * maxSpeed);

        // Generate command string
        char command[16];
        snprintf(command, 16, "#tnk(%d,%d)\n", leftInt, rightInt);

        // Write command to socket
        if (write(m_Socket, command, strlen(command)) < 0) {
            throw std::runtime_error("Cannot write to socket");
        }
    }

    virtual millimeter_t getRobotWidth() override
    {
        return 150_mm;
    }

private:
    //----------------------------------------------------------------------------
    // Private members
    //----------------------------------------------------------------------------
    int m_Socket;
}; // Surveyor
} // Robots
} // BoBRobotics
