#pragma once

// BoB robotics includes
#include "../common/background_exception_catcher.h"
#include "../common/logging.h"
#include "../common/semaphore.h"
#include "../net/connection.h"
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace BoBRobotics {
namespace Video {
using namespace std::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Video::NetSink
//----------------------------------------------------------------------------
//! Object for sending video frames synchronously or asynchronously over network
class NetSink
{
public:
    /*!
     * \brief Create a NetSink for asynchronous operation
     *
     * @param connection The connection over which to transmit images
     * @param input The Input source for images
     */
    NetSink(Net::Connection &connection, Input &input)
      : m_Connection(connection)
      , m_Name(input.getCameraName())
      , m_FrameSize(input.getOutputSize())
      , m_Input(&input)
    {
        // handle incoming IMG commands
        m_Connection.setCommandHandler("IMG",
                                       [this](Net::Connection &, const Net::Command &command) {
                                           onCommandReceivedAsync(command);
                                       });
    }

    /*!
     * \brief Create a NetSink for synchronous operation
     *
     * @param connections The connection over which to transmit images
     * @param frameSize The size of the frames output by the video source
     * @param cameraName The name of the camera (see Input::getCameraName())
     */
    NetSink(Net::Connection &connection, const cv::Size &frameSize, const std::string &cameraName)
      : m_Connection(connection)
      , m_Name(cameraName)
      , m_FrameSize(frameSize)
      , m_Input(nullptr)
    {
        // handle incoming IMG commands
        m_Connection.setCommandHandler("IMG",
                                       [this](Net::Connection &, const Net::Command &command) {
                                           onCommandReceivedSync(command);
                                       });
    }

    virtual ~NetSink()
    {
        LOG_DEBUG << "Waiting for Video::NetSink to finish...";

        // Ignore IMG commands
        m_Connection.setCommandHandler("IMG", nullptr);

        m_DoRun = false;
        if (m_Thread.joinable()) {
            m_Thread.join();
        }

        LOG_DEBUG << "Video::NetSink stopped";
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    //! Send a frame over the network (when operating in synchronous mode)
    void sendFrame(const cv::Mat &frame)
    {
        // Wait for start acknowledgement
        m_AckSemaphore.waitOnce();

        sendFrameInternal(frame);
    }

private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    void sendFrameInternal(const cv::Mat &frame)
    {
        cv::imencode(".jpg", frame, m_Buffer);

        auto socket = m_Connection.getSocketWriter();
        socket.send("IMG FRAME " + std::to_string(m_Buffer.size()) + "\n");
        socket.send(m_Buffer.data(), m_Buffer.size());
    }

    void onCommandReceived(const Net::Command &command)
    {
        if (command[1] != "START") {
            throw Net::BadCommandError();
        }

        // ACK the command and tell client the camera resolution
        m_Connection.getSocketWriter().send("IMG PARAMS " + std::to_string(m_FrameSize.width) + " " +
                                      std::to_string(m_FrameSize.height) + " " +
                                      m_Name + "\n");
    }

    void onCommandReceivedAsync(const Net::Command &command)
    {
        // Handle command
        onCommandReceived(command);

        // start thread to transmit images in background
        m_Thread = std::thread(&NetSink::runAsync, this);
    }

    void onCommandReceivedSync(const Net::Command &command)
    {
        // Handle command
        onCommandReceived(command);

        // Raise semaphore
        m_AckSemaphore.notify();
    }

    void runAsync()
    {
        try {
            cv::Mat frame;
            while (m_DoRun) {
                if (m_Input->readFrame(frame)) {
                    sendFrameInternal(frame);
                } else {
                    std::this_thread::sleep_for(25ms);
                }
            }
        } catch (...) {
            BackgroundExceptionCatcher::set(std::current_exception());
        }
    }

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Net::Connection &m_Connection;
    Semaphore m_AckSemaphore;
    const std::string m_Name;
    std::vector<uchar> m_Buffer;
    std::thread m_Thread;
    const cv::Size m_FrameSize;
    Input *m_Input;
    std::atomic<bool> m_DoRun{ true };
};
} // Video
} // BoBRobotics
