#pragma once

// C++ includes
#include <memory>
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "robots/motor.h"
#include "socket.h"
#include "video/input.h"

namespace GeNNRobotics {
namespace Net {
class Server;

class Server
{
public:
    static void runServer(Video::Input *videoinput, std::shared_ptr<Robots::Motor> &motor);

    Server(Video::Input *videoinput, std::shared_ptr<Robots::Motor> motor,
           int port = Socket::DefaultListenPort);
    virtual ~Server();

private:
    Video::Input *m_VideoInput;
    socket_t m_ListenSocket = INVALID_SOCKET;
    std::shared_ptr<Socket> m_Socket;
    std::shared_ptr<Robots::Motor> m_Motor;
    bool m_SendingImages = false;
    std::unique_ptr<std::thread> m_ImageThread;
    cv::Mat m_Frame;
    std::vector<uchar> m_FrameBuffer;

    bool parseCommand();
    void run();
    void sendFrame();
    static void runImageThread(Server *server);
};

/*
 * Create a server to send motor commands
 */
Server::Server(Video::Input *videoinput,
               std::shared_ptr<Robots::Motor> motor,
               int port)
  : m_VideoInput(videoinput)
  , m_Motor(motor)
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

/* Stop listening */
Server::~Server()
{
    if (m_SendingImages && m_ImageThread) {
        m_SendingImages = false;
        m_ImageThread->join();
    }

    if (m_ListenSocket != INVALID_SOCKET) {
        close(m_ListenSocket);
    }

    // needed for Windows
    WSACleanup();
}

void
Server::sendFrame()
{
    if (!m_VideoInput->readFrame(m_Frame)) {
        throw std::runtime_error("Could not read from camera");
    }

    cv::imencode(".jpg", m_Frame, m_FrameBuffer);
    m_Socket->send("IMG " + std::to_string(m_FrameBuffer.size()) + "\n");
    m_Socket->send(m_FrameBuffer.data(), m_FrameBuffer.size());
}

bool
Server::parseCommand()
{
    std::vector<std::string> command = m_Socket->readCommand();

    // driving command (e.g. TNK 0.5 0.5)
    if (command[0] == "TNK") {
        // second space separates left and right parameters
        if (command.size() != 3) {
            throw bad_command_error();
        }

        // parse strings to floats
        const float left = stof(command[1]);
        const float right = stof(command[2]);

        // send motor command
        m_Motor->tank(left, right);
        return true;
    } else if (command[0] == "IMS") {
        // ACK the command and tell client the camera resolution
        cv::Size res = m_VideoInput->getOutputSize();
        m_Socket->send("IMP " + std::to_string(res.width) + " " +
                       std::to_string(res.height) + "\n");

        m_SendingImages = true;
        m_ImageThread = std::unique_ptr<std::thread>(
                new std::thread(runImageThread, this));

        return true;
    } else if (command[0] == "BYE") {
        // client closing connection
        return false;
    }

    // no other commands supported
    throw bad_command_error();
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

    // loop for ever

    while (true) {
        // wait for incoming TCP connection
        std::cout << "Waiting for incoming connection..." << std::endl;
        m_Socket = std::shared_ptr<Socket>(new Socket(
                accept(m_ListenSocket, (sockaddr *) &addr, &addrlen)));
        m_Socket->send("HEY\n");

        // convert IP to string
        char saddr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, (void *) &addr.sin_addr, saddr, addrlen);
        std::cout << "Incoming connection from " << saddr << std::endl;

        try {
            while (parseCommand())
                ;
        } catch (socket_error &e) {
            std::cout << "Connection closed [" + std::string(e.what()) + "]"
                      << std::endl;
        }
    }
}

void
Server::runServer(Video::Input *videoinput, std::shared_ptr<Robots::Motor> &motor)
{
    Server server(videoinput, motor);
    server.run();
}

void
Server::runImageThread(Server *server)
{
    while (server->m_SendingImages) {
        server->sendFrame();
    }
}
} // Net
} // GeNNRobotics
