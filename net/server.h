#pragma once

// C++ includes
#include <memory>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../robots/tank.h"
#include "../video/input.h"

// local includes
#include "node.h"
#include "socket.h"

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
        struct sockaddr_in addr;
        int on = 1;

        m_ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (m_ListenSocket == INVALID_SOCKET) {
            goto error;
        }

#ifndef _WIN32
        if (setsockopt(m_ListenSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0) {
            goto error;
        }
#endif

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);

        if (bind(m_ListenSocket, (const sockaddr *) &addr, (int) sizeof(addr))) {
            goto error;
        }

        return;

    error:
        std::cerr << "Error (" << errno << "): Could not bind to port " << port
                  << std::endl;
        exit(1);
    }

    //! Stop listening and stop background thread
    virtual ~Server()
    {
        stop();

        if (m_ListenSocket != INVALID_SOCKET) {
            close(m_ListenSocket);
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

            try {
                Node::runInternal();
            } catch (SocketError &e) {
                std::cout << "Connection closed [" + std::string(e.what()) + "]"
                          << std::endl;
            }
        }
    }

private:
    socket_t m_ListenSocket = INVALID_SOCKET;
    std::unique_ptr<Socket> m_Socket;
};
} // Net
} // BoBRobotics
