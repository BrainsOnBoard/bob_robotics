#pragma once

// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "common/semaphore.h"
#include "robots/gazebo/node.h"
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
           const bool isPanoramic = false)
      : Camera(getNode(), topic, isPanoramic)
    {}

    /*!
     * \brief Create a video stream for a Gazebo topic using a user-provided tranport node
     *
     * @param node Gazebo transport node
     * @param topic Gazebo transport topic on which to subscribe
     */
    Camera(gazebo::transport::NodePtr node,
           const std::string &topic = DefaultTopic,
           const bool isPanoramic = false)
      : m_ImageNode(node)
      , m_IsPanoramic(isPanoramic)
    {
        // Subscribe to the topic, and register a callback
        m_ImageSub = m_ImageNode->Subscribe(topic, &Camera::onImageMsg, this);
        LOG_INFO << "Subsribed to " << topic;
    }

    // Public virtual methods
    virtual std::string getCameraName() const override
    {
        return m_IsPanoramic ? "gazebo_panoramic_camera" : "gazebo_camera";
    }

    virtual cv::Size getOutputSize() const override
    {
        m_HaveReceivedFrameSemaphore.waitOnce();
        return m_ReceivedImage.size();
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        if (!m_HaveReceivedFrames.load()) {
            return false;
        }

        std::lock_guard<std::mutex> lck(m_Mtx);
        m_ReceivedImage.copyTo(outFrame);

        // If there's no error, then we have updated frame and so return true
        return true;
    }

    virtual bool needsUnwrapping() const override
    {
        return m_IsPanoramic;
    }

private:
    gazebo::transport::SubscriberPtr m_ImageSub;
    gazebo::transport::NodePtr m_ImageNode;
    cv::Mat m_ReceivedImage;
    std::mutex m_Mtx;
    std::atomic<bool> m_HaveReceivedFrames{ false };
    mutable Semaphore m_HaveReceivedFrameSemaphore;
    bool m_IsPanoramic;
    static constexpr const char *DefaultTopic = "/gazebo/default/camera/link/camera/image";

    void onImageMsg(ConstImageStampedPtr &msg)
    {
        std::lock_guard<std::mutex> lck(m_Mtx);
        m_ReceivedImage.create(msg->image().height(), msg->image().width(), CV_8UC3);
        std::copy_n(msg->image().data().c_str(), msg->image().data().length(), m_ReceivedImage.data);
        if (!m_HaveReceivedFrames) {
            m_HaveReceivedFrames = true;
            m_HaveReceivedFrameSemaphore.notify();
        }
    }

}; // Camera
} // Gazebo
} // Robots
} // BoBRobotics
