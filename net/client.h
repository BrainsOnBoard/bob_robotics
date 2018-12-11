#pragma once

// BoB robotics includes
#include "connection.h"
#include "socket.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdint>

// Standard C++ includes
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
    Client(const std::string &host, uint16_t port = DefaultListenPort)
      : Connection(AF_INET, SOCK_STREAM, 0)
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
}; // Client
} // Net
} // BoBRobotics
