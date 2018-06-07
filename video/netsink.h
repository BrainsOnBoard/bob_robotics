#pragma once

// C++ includes
#include <future>
#include <mutex>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../net/node.h"

//----------------------------------------------------------------------------
// GeNNRobotics::Video::NetSink
//----------------------------------------------------------------------------
// Object for sending video frames synchronously or asynchronously over network
namespace GeNNRobotics {
namespace Video {
class NetSink
{
public:
    // Create a NetSink for asynchronous operation
    NetSink(Net::Node &node, Input &input)
    :   m_Node(node), m_Input(&input), m_FrameSize(input.getOutputSize()), m_Name(input.getCameraName())
    {
        // handle incoming IMG commands
        m_Node.addCommandHandler("IMG",
                                 [this](Net::Node &node, const Net::Command& command)
                                 {
                                     onCommandReceivedAsync(command);
                                 });
    }

    NetSink(Net::Node &node, const cv::Size &frameSize, const std::string &name)
    :   m_Node(node), m_Input(nullptr), m_FrameSize(frameSize), m_Name(name)
    {
        // handle incoming IMG commands
        m_Node.addCommandHandler("IMG",
                                 [this](Net::Node &node, const Net::Command& command)
                                 {
                                     onCommandReceivedSync(command);
                                 });
    }

    virtual ~NetSink()
    {
        if (m_ThreadRunning) {
            if (m_Thread.joinable()) {
                m_Thread.join();
            }
            else {
                m_Thread.detach();
            }
        }
    }
    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void sendFrame(const cv::Mat &frame)
    {
        // Wait for start acknowledgement
        m_AckPromise.get_future().wait();

        sendFrameInternal(frame);
    }



private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    void sendFrameInternal(const cv::Mat &frame)
    {
        cv::imencode(".jpg", frame, m_Buffer);
        m_Node.getSocket()->send("IMG FRAME " + std::to_string(m_Buffer.size()) + "\n");
        m_Node.getSocket()->send(m_Buffer.data(), m_Buffer.size());
    }

    void onCommandReceived(const Net::Command &command)
    {
        if (command[1] != "START") {
            throw Net::bad_command_error();
        }

        // ACK the command and tell client the camera resolution
        m_Node.getSocket()->send("IMG PARAMS " + std::to_string(m_FrameSize.width) + " " +
                                 std::to_string(m_FrameSize.height) + " " +
                                 m_Name + "\n");
    }

    void onCommandReceivedAsync(const Net::Command &command)
    {
        // Handle command
        onCommandReceived(command);

        // start thread to transmit images in background
        m_ThreadRunning = true;
        m_Thread = std::thread(&NetSink::runAsync, this);
    }

    void onCommandReceivedSync(const Net::Command &command)
    {
        // Handle command
        onCommandReceived(command);

        // Send future
        m_AckPromise.set_value();
    }

    void runAsync()
    {
        cv::Mat frame;
        while (m_ThreadRunning && m_Input->readFrame(frame)) {
            sendFrameInternal(frame);
        }
    }

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Net::Node &m_Node;
    Input *m_Input;
    const cv::Size m_FrameSize;
    const std::string m_Name;
    std::thread m_Thread;
    std::vector<uchar> m_Buffer;
    std::promise<void> m_AckPromise;
    bool m_ThreadRunning = false;
};
}   // Video
}   // GeNNRobotics