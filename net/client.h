/*
 * General-purpose TCP client, to be used with corresponding Net::Server object.
 * Various sink/source-type objects are used for either sending or receiving
 * data from the server.
 */

#pragma once

// C++ includes
#include <limits>
#include <string>
#include <vector>

// OpenCV
#include "../common/opencv.h"

// local includes
#include "node.h"
#include "socket.h"

namespace BoBRobotics {
namespace Net {
class Client
  : public Node
  , Socket
{
public:
    Client(const std::string &host, int port = DefaultListenPort);
    ~Client();
    Socket *getSocket() const override;
};

/*
 * Create client, connect to host over TCP
 */
Client::Client(const std::string &host, int port)
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
        throw std::runtime_error("Cannot connect socket to " + host + ":" +
                                 std::to_string(port));
    }

    std::cout << "Opened socket" << std::endl;

    notifyConnectedHandlers();
}

Client::~Client()
{
    stop(); // stop thread if needed
}

/*
 * Overridden Node method; used to get current socket, which for the Client
 * object is always itself.
 */
Socket *
Client::getSocket() const
{
    return (Socket *) this;
}
} // Net
} // BoBRobotics
