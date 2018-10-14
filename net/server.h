#pragma once

// BoB robotics includes
#include "../robots/tank.h"
#include "../video/input.h"
#include "node.h"
#include "socket.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <memory>
#include <exception>
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
class Server : public Node
{
public:
    //! Create a new server, listening on the specified port
    Server(int port = Socket::DefaultListenPort)
    {
        try {
            m_ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
            if (m_ListenSocket == INVALID_SOCKET) {
                throw std::exception();
            }

    #ifndef _WIN32
            int on = 1;
            if (setsockopt(m_ListenSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
                throw std::exception();
            }
    #endif

            struct sockaddr_in addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons(port);

            if (bind(m_ListenSocket, (const sockaddr *) &addr, (int) sizeof(addr))) {
                throw std::exception();
            }
        } catch (...) {
            throw SocketError("Could not bind to port");
        }
    }

    //! Get the socket associated with the current connection
    Socket *getSocket() const override
    {
        return m_Socket.get();
    }

protected:
    virtual void runInternal() override
    {
        // Start listening
        if (listen(m_ListenSocket, 10)) {
            throw std::runtime_error("Error (" + std::to_string(errno) + "): Could not listen");
        }

        // for incoming connection
        sockaddr_in addr;
        socklen_t addrlen = sizeof(addr);

        // loop until stopped
        while (isRunning()) {
            // wait for incoming TCP connection
            std::cout << "Waiting for incoming connection..." << std::endl;
            m_Socket = std::make_unique<Socket>(accept(m_ListenSocket, (sockaddr *) &addr, &addrlen));
            m_Socket->send("HEY\n");
            notifyConnectedHandlers();

            // convert IP to string
            char saddr[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, (void *) &addr.sin_addr, saddr, addrlen);
            std::cout << "Incoming connection from " << saddr << std::endl;

            // read incoming commands in a loop
            Node::runInternal();
        }
    }

private:
    socket_t m_ListenSocket = INVALID_SOCKET;
    std::unique_ptr<Socket> m_Socket;
};
} // Net
} // BoBRobotics
