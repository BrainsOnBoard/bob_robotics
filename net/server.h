#pragma once

// C++ includes
#include <memory>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../robots/motor.h"
#include "../video/input.h"

// local includes
#include "node.h"
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
class Server : public Node
{
public:
    Server(int port = Socket::DefaultListenPort);
    virtual ~Server();
    Socket *getSocket() const override;
    void run() override;

protected:

private:
    socket_t m_ListenSocket = INVALID_SOCKET;
    std::unique_ptr<Socket> m_Socket;
};

/*
 * Create a server to send motor commands
 */
Server::Server(int port)
{
    struct sockaddr_in addr;
    int on = 1;

    // needed for Windows
    WSAStartup();

    m_ListenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_ListenSocket == INVALID_SOCKET) {
        goto error;
    }

#ifndef _WIN32
    if (setsockopt(m_ListenSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) <
        0) {
        goto error;
    }
#endif

    memset(&addr, '0', sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(m_ListenSocket, (const sockaddr *) &addr, (int) sizeof(addr))) {
        goto error;
    }
    if (listen(m_ListenSocket, 10)) {
        goto error;
    }
    std::cout << "Listening on port " << port << std::endl;

    return;

error:
    std::cerr << "Error (" << errno << "): Could not bind to port " << port
              << std::endl;
    exit(1);
}

Server::~Server()
{
    stop();

    if (m_ListenSocket != INVALID_SOCKET) {
        close(m_ListenSocket);
    }

    // needed for Windows
    WSACleanup();
}

/*
 * Get the socket of the current connection.
 */
Socket *
Server::getSocket() const
{
    return m_Socket.get();
}

/*
 * Keep accepting connections and parsing input for ever. Can only handle
 * one connection at a time.
 */
void
Server::run()
{
    // for incoming connection
    sockaddr_in addr;
    socklen_t addrlen = sizeof(addr);

    // loop until stopped
    while (m_DoRun) {
        // wait for incoming TCP connection
        std::cout << "Waiting for incoming connection..." << std::endl;
        m_Socket = std::unique_ptr<Socket>(new Socket(
                accept(m_ListenSocket, (sockaddr *) &addr, &addrlen)));
        m_Socket->send("HEY\n");
        notifyConnectedHandlers();

        // convert IP to string
        char saddr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, (void *) &addr.sin_addr, saddr, addrlen);
        std::cout << "Incoming connection from " << saddr << std::endl;

        try {
            Node::run();
        } catch (socket_error &e) {
            std::cout << "Connection closed [" + std::string(e.what()) + "]"
                      << std::endl;
        }
    }
}
} // Net
} // GeNNRobotics
