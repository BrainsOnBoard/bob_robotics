#pragma once

// BoB robotics includes
#include "common/semaphore.h"
#include "net/connection.h"
#include "input.h"

// Standard C++ includes
#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace BoBRobotics {
namespace Video {
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
    NetSink(Net::Connection &connection, Input &input);

    /*!
     * \brief Create a NetSink for synchronous operation
     *
     * @param connections The connection over which to transmit images
     * @param frameSize The size of the frames output by the video source
     * @param cameraName The name of the camera (see Input::getCameraName())
     */
    NetSink(Net::Connection &connection,
            const cv::Size &frameSize,
            const std::string &cameraName);

    virtual ~NetSink();

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    //! Send a frame over the network (when operating in synchronous mode)
    void sendFrame(const cv::Mat &frame);

private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    void sendFrameInternal(const cv::Mat &frame);

    void onCommandReceived(const Net::Command &command);

    void onCommandReceivedAsync(const Net::Command &command);

    void onCommandReceivedSync(const Net::Command &command);

    void runAsync();

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
