#pragma once

// C++ includes
#include <string>
#include <thread>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../net/node.h"
#include "../net/socket.h"

// local includes
#include "input.h"

namespace GeNNRobotics {
namespace Video {
class NetSink : public Net::Handler
{
public:
    NetSink(Input *input)
      : m_Input(input)
    {}

    void onCommandReceived(Net::Node &node, Net::Command &command)
    {
        if (command[1] != "START") {
            throw Net::bad_command_error();
        }

        // ACK the command and tell client the camera resolution
        cv::Size res = m_Input->getOutputSize();
        node.getSocket()->send("IMG PARAMS " + std::to_string(res.width) + " " +
                               std::to_string(res.height) + "\n");

        // start thread to transmit images in background
        m_ImageThread = std::unique_ptr<std::thread>(
                new std::thread([=](Net::Node *n) { runImageSink(n); }, &node));
    }

    std::string getHandledCommandName() const
    {
        return "IMG";
    }

private:
    Input *m_Input;
    std::unique_ptr<std::thread> m_ImageThread;

    void runImageSink(Net::Node *node)
    {
        cv::Mat frame;
        std::vector<uchar> buffer;
        Net::Socket *sock = node->getSocket();
        while (m_Input->readFrame(frame)) {
            cv::imencode(".jpg", frame, buffer);
            sock->send("IMG FRAME " + std::to_string(buffer.size()) + "\n");
            sock->send(buffer.data(), buffer.size());
        }
    }
};
}
}
