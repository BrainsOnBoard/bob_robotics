#pragma once

// BoB robotics includes
#include "../common/semaphore.h"
#include "../net/connection.h"
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
    NetSource(Net::Connection &connection)
      : m_Connection(connection)
    {
        // Handle incoming IMG commands
        connection.setCommandHandler("IMG", [this](Net::Connection &connection, const Net::Command &command) {
            onCommandReceived(connection, command);
        });

        // When connected, send command to start streaming
        connection.getSocketWriter().send("IMG START\n");
    }

    virtual ~NetSource() override
    {
        // Ignore IMG commands
        m_Connection.setCommandHandler("IMG", nullptr);
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
    cv::Mat m_Frame;
    mutable Semaphore m_ParamsSemaphore;
    std::vector<uchar> m_Buffer;
    std::string m_CameraName = DefaultCameraName;
    Net::Connection &m_Connection;
    cv::Size m_CameraResolution;
    std::mutex m_FrameMutex;
    std::atomic<bool> m_NewFrame{ false };

    void onCommandReceived(Net::Connection &connection, const Net::Command &command)
    {
        if (command[1] == "PARAMS") {
            m_CameraResolution.width = stoi(command[2]);
            m_CameraResolution.height = stoi(command[3]);
            m_CameraName = command[4];
            m_ParamsSemaphore.notify();
        } else if (command[1] == "FRAME") {
            const auto nbytes = static_cast<size_t>(stoul(command[2]));
            m_Buffer.resize(nbytes);
            connection.read(m_Buffer.data(), nbytes);

            std::lock_guard<std::mutex> guard(m_FrameMutex);
            cv::imdecode(m_Buffer, cv::IMREAD_UNCHANGED, &m_Frame);
            m_NewFrame = true;
        } else {
            throw Net::BadCommandError();
        }
    }
}; // NetSource
} // Video
} // BoBRobotics
