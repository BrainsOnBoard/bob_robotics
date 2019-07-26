// BoB robotics includes
#include "common/logging.h"
#include "net/server.h"

// Standard C includes
#include <cstring>

namespace BoBRobotics {
namespace Net {

Server::Server(uint16_t port)
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

Connection
Server::waitForConnection() const
{
    // For address of incoming connection
    sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);

    // Wait for incoming TCP connection
    LOG_INFO << "Waiting for incoming connection...";
    Socket socket(accept(m_ListenSocket.getHandle(), (sockaddr *) &addr, &addrlen));
    socket.send("HEY\n");

    // Convert IP to string
    char saddr[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, (void *) &addr.sin_addr, saddr, addrlen);
    LOG_INFO << "Incoming connection from " << saddr;

    return Connection(std::move(socket));
}

} // Net
} // BoBRobotics
