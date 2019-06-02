#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "input.h"

// Gazebo includes
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/common.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <stdexcept>
#include <mutex>
#include <atomic>

namespace BoBRobotics {
namespace Video {

constexpr const char *GazeboCameraDeviceName = "gazebo_camera";

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
    GazeboCameraInput(const std::string &topic, const std::string &cameraName = GazeboCameraDeviceName)
    : m_CameraTopic(topic), m_CameraName(cameraName)
    {
        // Load gazebo as a client
        #if GAZEBO_MAJOR_VERSION < 6
        gazebo::setupClient(0, 0);
        #else
        gazebo::client::setup(0, 0);
        #endif
        //Setup image subscription node
        m_ImageNode= gazebo::transport::NodePtr(new gazebo::transport::Node());
        m_ImageNode->Init();
        subscribeToGazeboCamera();
    }
    /*!
     * \brief Create a video stream for a Gazebo topic using a user-provided tranport node
     *
     * @param node Gazebo transport node
     * @param topic Gazebo transport topic on which to subscribe
     */
    
    GazeboCameraInput(gazebo::transport::NodePtr node, const std::string &topic="/gazebo/default/camera/link/camera/image", const std::string &cameraName = GazeboCameraDeviceName)
      : m_CameraTopic(topic), m_CameraName(cameraName), m_ImageNode(node)
    {
        subscribeToGazeboCamera();
    }

    void subscribeToGazeboCamera(){
        // Subscribe to the topic, and register a callback
        m_HaveReceivedFrames = ATOMIC_VAR_INIT(false);
        m_ImageSub = m_ImageNode->Subscribe(m_CameraTopic, &GazeboCameraInput::OnImageMsg, this);
        LOG_INFO << "Subsribed to "<< m_CameraTopic <<"\n";
    }
    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override
    {
        return "gazebo_camera";
    }

    virtual cv::Size getOutputSize() const override
    {
        return m_OutSize;
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        if(!m_HaveReceivedFrames.load())
            return false;
        std::lock_guard<std::mutex> lck(m_Mtx);
        m_ReceivedImage = cv::Mat(m_OutSize.height, m_OutSize.width, CV_8UC3, m_Data);
        outFrame = m_ReceivedImage;
        // If there's no error, then we have updated frame and so return true
        return true;
    }


private:
    const std::string m_CameraTopic;
    std::string m_CameraName;
    gazebo::transport::SubscriberPtr m_ImageSub;
    gazebo::transport::NodePtr m_ImageNode;
    cv::Size m_OutSize;
    char *m_Data;
    cv::Mat m_ReceivedImage;
    std::mutex m_Mtx;
    std::atomic<bool> m_HaveReceivedFrames;

    void OnImageMsg(ConstImageStampedPtr &msg)
    {
        std::lock_guard<std::mutex> lck(m_Mtx);

        if(!m_HaveReceivedFrames.load()){
            // Assuming image size remains constant at runtime, we can allocate memory for m_Data only once
            m_OutSize.width = (int) msg->image().width();
            m_OutSize.height = (int) msg->image().height();
            m_Data = new char[msg->image().data().length() + 1];
            m_HaveReceivedFrames.store(true);
        }    
        memcpy(m_Data, msg->image().data().c_str(), msg->image().data().length());
    }
}; // GazeboCameraInput
} // Video
} // BoBRobotics
