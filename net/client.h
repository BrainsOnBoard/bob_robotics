#pragma once

// C++ includes
#include <limits>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// local includes
#include "node.h"
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
class Client
  : public Node
  , Socket
{
public:
    Client(const std::string host);
    ~Client();
    Socket *getSocket() const override;

private:
    cv::Size m_CameraResolution;
    float m_OldLeft = std::numeric_limits<float>::quiet_NaN();
    float m_OldRight = std::numeric_limits<float>::quiet_NaN();
    std::vector<uchar> m_FrameBuffer;
};

/* Create client, connect to host on MAIN_PORT over TCP */
Client::Client(const std::string host)
{
    WSAStartup();

    // Create socket
    setSocket(socket(AF_INET, SOCK_STREAM, 0));

    // Create socket address structure
    in_addr addr;
    addr.s_addr = inet_addr(host.c_str());
    sockaddr_in destAddress;
    destAddress.sin_family = AF_INET;
    destAddress.sin_port = htons(DefaultListenPort);
    destAddress.sin_addr = addr;

    // Connect socket
    if (connect(Socket::getSocket(),
                reinterpret_cast<sockaddr *>(&destAddress),
                sizeof(destAddress)) < 0) {
        throw std::runtime_error("Cannot connect socket to " + host + ":" +
                                 std::to_string(DefaultListenPort));
    }

    std::cout << "Opened socket" << std::endl;
}

Client::~Client()
{
    stop();
    WSACleanup();
}

Socket *
Client::getSocket() const
{
    return (Socket *) this;
}
} // Net
} // GeNNRobotics
