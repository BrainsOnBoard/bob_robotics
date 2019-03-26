#pragma once

// BoB robotics includes
#include "connection.h"
#include "socket.h"

// Standard C includes
#include <cstdint>
#include <cstdlib>

// Standard C++ includes
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Net {
//----------------------------------------------------------------------------
// BoBRobotics::Net::Client
//----------------------------------------------------------------------------
/*!
 * \brief General-purpose TCP client
 *
 * To be used with corresponding Server object. Various sink/source-type
 * objects are used for either sending or receiving data from the server.
 */
class Client
  : public Connection
{
public:
    //! Create client and connect to host over TCP
    Client(const std::string &host = getDefaultIP(), uint16_t port = DefaultListenPort)
      : Connection(AF_INET, SOCK_STREAM, 0)
      , m_IP(host)
    {
        // Create socket address structure
        in_addr addr;
        addr.s_addr = inet_addr(host.c_str());
        sockaddr_in destAddress;
        destAddress.sin_family = AF_INET;
        destAddress.sin_port = htons(port);
        destAddress.sin_addr = addr;

        // Connect socket
        if (connect(getSocket().getHandle(),
                    reinterpret_cast<sockaddr *>(&destAddress),
                    sizeof(destAddress)) < 0) {
            getSocket().close();
            throw OS::Net::NetworkError("Cannot connect socket to " + host + ":" +
                                        std::to_string(port));
        }

        std::cout << "Opened socket" << std::endl;
    }

    const std::string &getIP() const { return m_IP; }

    static std::string getDefaultIP()
    {
        constexpr const char *envVar = "ROBOT_IP", *defaultIP = "127.0.0.1";

        const char *ip = std::getenv(envVar);
        if (!ip) {
            std::cerr << "WARNING: Environment variable " << envVar
                      << " not set; using " << defaultIP << std::endl;
            ip = defaultIP;
        }
        return ip;
    }

    private:
        const std::string m_IP;
}; // Client
} // Net
} // BoBRobotics
