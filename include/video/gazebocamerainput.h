#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/gazebo_node.h"
#include "input.h"
#include "common/semaphore.h"

// Gazebo includes
#include <gazebo/transport/transport.hh>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <stdexcept>
#include <mutex>
#include <atomic>
#include <semaphore.h>

namespace BoBRobotics {
namespace Video {


//----------------------------------------------------------------------------
// BoBRobotics::Video::GazeboCameraInput
//----------------------------------------------------------------------------
//! A thin wrapper for reading from any Gazebo camera feed
class GazeboCameraInput : public Input
{
public:
    /*!
     * \brief Create a video stream using a new transport node and default topic
     */
    GazeboCameraInput()
    : GazeboCameraInput("/gazebo/default/camera/link/camera/image")
    {}
    /*!
     * \brief Create a video stream using a new transport node
     */
    GazeboCameraInput(const std::string &topic, const std::string &cameraName = DefaultCameraName)
    : m_CameraName(cameraName)
    {
        m_ImageNode = getGazeboNode();
        subscribeToGazeboCamera(topic);
    }
    /*!
     * \brief Create a video stream for a Gazebo topic using a user-provided tranport node
     *
     * @param node Gazebo transport node
     * @param topic Gazebo transport topic on which to subscribe
     */

    GazeboCameraInput(gazebo::transport::NodePtr node, const std::string &topic="/gazebo/default/camera/link/camera/image", const std::string &cameraName = DefaultCameraName)
      : m_CameraName(cameraName), m_ImageNode(node)
    {
        subscribeToGazeboCamera(topic);
    }

    void subscribeToGazeboCamera(const std::string &topic){
        // Subscribe to the topic, and register a callback
        m_ImageSub = m_ImageNode->Subscribe(topic, &GazeboCameraInput::OnImageMsg, this);
        LOG_INFO << "Subsribed to "<< topic <<"\n";
        m_HaveReceivedFrameSemaphore.wait();
    }
    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override
    {
        return m_CameraName;
    }

    virtual cv::Size getOutputSize() const override
    {
        return m_ReceivedImage.size();
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        if(!m_HaveReceivedFrames.load())
            return false;
        std::lock_guard<std::mutex> lck(m_Mtx);
        outFrame = m_ReceivedImage;
        // If there's no error, then we have updated frame and so return true
        return true;
    }


private:
    std::string m_CameraName;
    gazebo::transport::SubscriberPtr m_ImageSub;
    gazebo::transport::NodePtr m_ImageNode;
    cv::Mat m_ReceivedImage;
    std::mutex m_Mtx;
    std::atomic<bool> m_HaveReceivedFrames{ false };
    Semaphore m_HaveReceivedFrameSemaphore;

    void OnImageMsg(ConstImageStampedPtr &msg)
    {
        std::lock_guard<std::mutex> lck(m_Mtx);

        if(!m_HaveReceivedFrames.load()){
            m_HaveReceivedFrames.store(true);
            m_HaveReceivedFrameSemaphore.notify();
        }
        m_ReceivedImage.create(msg->image().height(), msg->image().width(), CV_8UC3);
        memcpy(m_ReceivedImage.data, msg->image().data().c_str(), msg->image().data().length());
    }
}; // GazeboCameraInput
} // Video
} // BoBRobotics
