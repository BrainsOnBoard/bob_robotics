#pragma once

// BoB robotics includes
#include "common/semaphore.h"
#include "net/connection.h"
#include "input.h"

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
    NetSource(Net::Connection &connection);

    virtual ~NetSource() override;

    virtual std::string getCameraName() const override;

    virtual cv::Size getOutputSize() const override;

    virtual bool needsUnwrapping() const override;

    virtual bool readFrame(cv::Mat &frame) override;

private:
    cv::Mat m_Frame;
    mutable Semaphore m_ParamsSemaphore;
    std::vector<uchar> m_Buffer;
    std::string m_CameraName = DefaultCameraName;
    Net::Connection &m_Connection;
    cv::Size m_CameraResolution;
    std::mutex m_FrameMutex;
    std::atomic<bool> m_NewFrame{ false };

    void onCommandReceived(Net::Connection &connection,
                           const Net::Command &command);
}; // NetSource
} // Video
} // BoBRobotics
