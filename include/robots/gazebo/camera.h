#pragma once

// BoB robotics includes
#include "common/semaphore.h"
#include "video/input.h"

// Gazebo includes
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <atomic>
#include <mutex>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

//----------------------------------------------------------------------------
// BoBRobotics::Robots::Gazebo::Camera
//----------------------------------------------------------------------------
//! A thin wrapper for reading from any Gazebo camera feed
class Camera
  : public Video::Input
{
public:
    /*!
     * \brief Create a video stream using a new transport node
     */
    Camera(const std::string &topic = DefaultTopic,
           const bool isPanoramic = false);

    /*!
     * \brief Create a video stream for a Gazebo topic using a user-provided tranport node
     *
     * @param node Gazebo transport node
     * @param topic Gazebo transport topic on which to subscribe
     */
    Camera(gazebo::transport::NodePtr node,
           const std::string &topic = DefaultTopic,
           const bool isPanoramic = false);

    // Public virtual methods
    virtual std::string getCameraName() const override;
    virtual cv::Size getOutputSize() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;
    virtual bool needsUnwrapping() const override;

private:
    gazebo::transport::SubscriberPtr m_ImageSub;
    gazebo::transport::NodePtr m_ImageNode;
    cv::Mat m_ReceivedImage;
    std::mutex m_Mtx;
    std::atomic<bool> m_HaveReceivedFrames{ false };
    mutable Semaphore m_HaveReceivedFrameSemaphore;
    bool m_IsPanoramic;
    static constexpr const char *DefaultTopic = "/gazebo/default/camera/link/camera/image";

    void onImageMsg(ConstImageStampedPtr &msg);

}; // Camera
} // Gazebo
} // Robots
} // BoBRobotics
