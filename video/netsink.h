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
class NetSink
{
public:
    NetSink(Input *input, Net::Node *node)
      : m_Input(input)
      , m_Node(node)
    {
        node->addHandler("IMG", handleCommand, this);
    }

private:
    Input *m_Input;
    std::unique_ptr<std::thread> m_ImageThread;
    Net::Node *m_Node;

    void handleCommand(std::vector<std::string> &command)
    {
        // ACK the command and tell client the camera resolution
        cv::Size res = m_Input->getOutputSize();
        m_Node->getSocket()->send("IMG PARAMS " + std::to_string(res.width) + " " +
                       std::to_string(res.height) + "\n");

        m_ImageThread = std::unique_ptr<std::thread>(
                new std::thread([=] { runImageSink(); }));
    }

    static void handleCommand(std::vector<std::string> &command, void *userData)
    {
        if (command[1] != "START") {
            throw Net::bad_command_error();
        }

        reinterpret_cast<NetSink *>(userData)->handleCommand(command);
    }

    void runImageSink()
    {
        cv::Mat frame;
        std::vector<uchar> buffer;
        Net::Socket *sock = m_Node->getSocket();
        while (m_Input->readFrame(frame)) {
            cv::imencode(".jpg", frame, buffer);
            sock->send("IMG FRAME " + std::to_string(buffer.size()) + "\n");
            sock->send(buffer.data(), buffer.size());
        }
    }
};
}
}
