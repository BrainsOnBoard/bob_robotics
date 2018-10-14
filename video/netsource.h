#pragma once

// BoB robotics includes
#include "../common/semaphore.h"
#include "../net/node.h"
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Video {
//----------------------------------------------------------------------------
// BoBRobotics::Video::NetSource
//----------------------------------------------------------------------------
//! Object for receiving video transmitted over the network by a NetSink
class NetSource : public Input
{
public:
    /*!
     * \brief Create an object to read video transmitted over the network
     *
     * @param node The network connection from which to read images
     */
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

    virtual std::string getCameraName() const override
    {
        m_ParamsSemaphore.waitOnce();
        return m_CameraName;
    }

    virtual cv::Size getOutputSize() const override
    {
        m_ParamsSemaphore.waitOnce();
        return m_CameraResolution;
    }

    virtual bool needsUnwrapping() const override
    {
        m_ParamsSemaphore.waitOnce();
        return Input::needsUnwrapping();
    }

    virtual bool readFrame(cv::Mat &frame) override
    {
        if (!m_NewFrame.exchange(false)) {
            // The return value indicates whether there is a new frame or not
            return false;
        } else {
            std::lock_guard<std::mutex> guard(m_FrameMutex);

            // Copy latest frame and return true
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
    std::atomic<bool> m_NewFrame{ false };
    mutable Semaphore m_ParamsSemaphore;

    void onCommandReceived(Net::Node &node, const Net::Command &command)
    {
        if (command[1] == "PARAMS") {
            m_CameraResolution.width = stoi(command[2]);
            m_CameraResolution.height = stoi(command[3]);
            m_CameraName = command[4];
            m_ParamsSemaphore.notify();
        } else if (command[1] == "FRAME") {
            size_t nbytes = stoi(command[2]);
            m_Buffer.resize(nbytes);
            node.getSocket()->read(m_Buffer.data(), nbytes);
            {
                std::lock_guard<std::mutex> guard(m_FrameMutex);
                cv::imdecode(m_Buffer, cv::IMREAD_UNCHANGED, &m_Frame);
                m_NewFrame = true;
            }
        } else {
            throw Net::BadCommandError();
        }
    }
}; // NetSource
} // Video
} // BoBRobotics
