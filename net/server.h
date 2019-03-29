#pragma once

// BoB robotics includes
#include "../common/logging.h"
#include "connection.h"
#include "socket.h"

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Net {
//----------------------------------------------------------------------------
// BoBRobotics::Net::Server
//----------------------------------------------------------------------------
/*!
 * \brief A general-purpose TCP server
 *
 * To be used with corresponding Client object. Various sink/source-type
 * objects are used for either sending or receiving data to the client.
 */
class Server
{
public:
    //! Create a new server, listening on the specified port
    Server(uint16_t port = Connection::DefaultListenPort)
      : m_ListenSocket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
    {
#ifndef _WIN32
        int on = 1;
        if (setsockopt(m_ListenSocket.getHandle(), SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
            throw OS::Net::NetworkError("Could not set socket option");
        }
#endif

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (bind(m_ListenSocket.getHandle(), (const sockaddr *) &addr, (int) sizeof(addr))) {
            throw OS::Net::NetworkError("Could not bind to socket");
        }

        // Start listening
        if (listen(m_ListenSocket.getHandle(), 10)) {
            throw OS::Net::NetworkError("Error while listening for connection");
        }
    }

    Connection waitForConnection() const
    {
        // For address of incoming connection
        sockaddr_in addr;
        socklen_t addrlen = sizeof(addr);

        // Wait for incoming TCP connection
        LOG_INFO << "Waiting for incoming connection..." << std::endl;
        Socket socket(accept(m_ListenSocket.getHandle(), (sockaddr *) &addr, &addrlen));
        socket.send("HEY\n");

        // Convert IP to string
        char saddr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, (void *) &addr.sin_addr, saddr, addrlen);
        LOG_INFO << "Incoming connection from " << saddr << std::endl;

        return Connection(std::move(socket));
    }

private:
    const Socket m_ListenSocket;
};
} // Net
} // BoBRobotics
