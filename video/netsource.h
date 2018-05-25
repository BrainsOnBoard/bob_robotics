#pragma once

// C++ includes
#include <mutex>
#include <string>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../common/semaphore.h"
#include "../net/node.h"

// local includes
#include "input.h"

namespace GeNNRobotics {
namespace Video {
class NetSource : public Input
{
public:
    cv::Size getOutputSize() const override
    {
        return m_CameraResolution;
    }

    bool readFrame(cv::Mat &frame) override
    {
        m_Semaphore.wait();
        std::lock_guard<std::mutex> guard(m_BufferMutex);
        cv::imdecode(m_Buffer, cv::IMREAD_UNCHANGED, &frame);
        return true;
    }

private:
    cv::Size m_CameraResolution;
    std::vector<uchar> m_Buffer;
    std::mutex m_BufferMutex;
    Semaphore m_Semaphore;

    void onConnected(Net::Node &node) override
    {
        node.getSocket()->send("IMG START\n");
    }

    void onCommandReceived(Net::Node &node, Net::Command &command) override
    {
        if (command[1] == "PARAMS") {
            m_CameraResolution.width = stoi(command[2]);
            m_CameraResolution.height = stoi(command[3]);
        } else if (command[1] == "FRAME") {
            std::lock_guard<std::mutex> guard(m_BufferMutex);
            size_t nbytes = stoi(command[2]);
            m_Buffer.resize(nbytes);
            node.getSocket()->read(m_Buffer.data(), nbytes);
            m_Semaphore.notify();
        } else {
            throw Net::bad_command_error();
        }
    }

    std::string getHandledCommandName() const override
    {
        return "IMG";
    }
}; // NetSource
} // Video
} // GeNNRobotics
