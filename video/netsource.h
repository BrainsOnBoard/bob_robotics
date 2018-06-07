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

// local includes
#include "input.h"

namespace GeNNRobotics {
namespace Video {
class NetSource : public Input
{
public:
    NetSource()
    {}

    NetSource(Net::Node &node)
    {
        // handle incoming IMG commands
        node.addCommandHandler("IMG", [this](Net::Node &node, const Net::Command &command) {
            onCommandReceived(node, command);
        });

        // when connected, send command to start streaming
        node.addConnectedHandler([](Net::Node &node) {
            node.getSocket()->send("IMG START\n");
        });
    }

    const std::string getCameraName() const override
    {
        return m_CameraName;
    }

    cv::Size getOutputSize() const override
    {
        return m_CameraResolution;
    }

    bool needsUnwrapping() const override
    {
        m_ParamsPromise.get_future().wait();
        return Input::needsUnwrapping();
    }

    bool readFrame(cv::Mat &frame) override
    {
        std::lock_guard<std::mutex> guard(m_FrameMutex);

        // If no frames have been read, return false
        if (m_Frame.cols == 0 && m_Frame.rows == 0) {
            return false;
        }
        // Otherwise, copy latest frame to buffer and return true
        else {
            m_Frame.copyTo(frame);
            return true;
        }
    }

private:
    std::string m_CameraName = DefaultCameraName;
    cv::Size m_CameraResolution;
    std::vector<uchar> m_Buffer;
    cv::Mat m_Frame;
    std::mutex m_FrameMutex;
    mutable std::promise<void> m_ParamsPromise;

    void onCommandReceived(Net::Node &node, const Net::Command &command)
    {
        if (command[1] == "PARAMS") {
            m_CameraResolution.width = stoi(command[2]);
            m_CameraResolution.height = stoi(command[3]);
            m_CameraName = command[4];
            m_ParamsPromise.set_value();
        } else if (command[1] == "FRAME") {
            std::lock_guard<std::mutex> guard(m_FrameMutex);
            size_t nbytes = stoi(command[2]);
            m_Buffer.resize(nbytes);
            node.getSocket()->read(m_Buffer.data(), nbytes);
            cv::imdecode(m_Buffer, cv::IMREAD_UNCHANGED, &m_Frame);
        } else {
            throw Net::bad_command_error();
        }
    }
}; // NetSource
} // Video
} // GeNNRobotics
