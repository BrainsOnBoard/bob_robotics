#pragma once

// BoB robotics includes
#include "node.h"
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
  : public Node
  , public Socket
{
public:
    //! Create client and connect to host over TCP
    Client(const std::string &host, uint16_t port = DefaultListenPort)
    {
        // Create socket
        setSocket(socket(AF_INET, SOCK_STREAM, 0));

        // Create socket address structure
        in_addr addr;
        addr.s_addr = inet_addr(host.c_str());
        sockaddr_in destAddress;
        destAddress.sin_family = AF_INET;
        destAddress.sin_port = htons(port);
        destAddress.sin_addr = addr;

        // Connect socket
        if (connect(Socket::getSocket(),
                    reinterpret_cast<sockaddr *>(&destAddress),
                    sizeof(destAddress)) < 0) {
            throw OS::Net::NetworkError("Cannot connect socket to " + host + ":" +
                                        std::to_string(port));
        }

        std::cout << "Opened socket" << std::endl;
        notifyConnectedHandlers();
    }

    virtual ~Client() override
    {
        disconnect();
    }

    //! Used to get current socket, which for the Client object is always itself
    virtual Socket *getSocket() override
    {
        return this;
    }
}; // Client
} // Net
} // BoBRobotics
