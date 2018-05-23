#pragma once

// C++ includes
#include <limits>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "robots/motor.h"
#include "video/input.h"

// local includes
#include "node.h"
#include "socket.h"

namespace GeNNRobotics {
namespace Net {
class Client
  : public Node
  , public Robots::Motor
  , public Video::Input
  , Socket
{
public:
    Client(const std::string host);
    ~Client();
    void startStreaming();
    void tank(float left, float right) override;
    cv::Size getOutputSize() const override;
    bool readFrame(cv::Mat &frame) override;
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

    // first command is HEY
    readLine();
}

/* Destructor: send BYE message and close connection */
Client::~Client()
{
    WSACleanup();
}

Socket *
Client::getSocket() const
{
    return nullptr;
}

void
Client::startStreaming()
{
    send("IMG START\n");
    auto command = readCommand();
    if (command[0] != "IMG" || command[1] != "PARAMS" || command.size() != 4) {
        throw bad_command_error();
    }

    m_CameraResolution.width = stoi(command[2]);
    m_CameraResolution.height = stoi(command[3]);
}

cv::Size
Client::getOutputSize() const
{
    return m_CameraResolution;
}

bool
Client::readFrame(cv::Mat &frame)
{
    auto command = readCommand();
    if (command[0] != "IMG" || command.size() != 2) {
        throw bad_command_error();
    }

    size_t nbytes = stoi(command[1]);
    m_FrameBuffer.resize(nbytes);
    read(m_FrameBuffer.data(), nbytes);
    cv::imdecode(m_FrameBuffer, cv::IMREAD_UNCHANGED, &frame);
    return true;
}

/* Motor command: send TNK command over TCP */
void
Client::tank(float left, float right)
{
    // don't send a command if it's the same as the last one
    if (left == m_OldLeft && right == m_OldRight) {
        return;
    }

    // send steering command
    send("TNK " + std::to_string(left) + " " + std::to_string(right) + "\n");

    // store current left/right values to compare next time
    m_OldLeft = left;
    m_OldRight = right;
}
} // Net
} // GeNNRobotics
