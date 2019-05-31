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

namespace BoBRobotics {
namespace Video {

constexpr const char *GazeboCameraDeviceName = "Gazebo Camera";

//----------------------------------------------------------------------------
// BoBRobotics::Video::GazeboCameraInput
//----------------------------------------------------------------------------
//! A thin wrapper for reading from any video source supported by OpenCV
class GazeboCameraInput : public Input
{
public:
    //! Create a new video stream
    GazeboCameraInput()
      : GazeboCameraInput()
    {}

    /*!
     * \brief Create a video stream for a specific device
     *
     * @param device Integer or string representation of device (passed to
     *        cv::VideoCapture's constructor)
     * @param cameraName The short name to use for this camera (see getCameraName())
     */
    
    GazeboCameraInput(const std::string &topic="/gazebo/default/camera/link/camera/image", const std::string &cameraName = GazeboCameraDeviceName)
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
        // Subscribe to the topic, and register a callback
        m_ImageSub = m_ImageNode->Subscribe(m_CameraTopic, &GazeboCameraInput::OnImageMsg, this);
        // m_OutSize.width = 320;
        // m_OutSize.height = 240;

        LOG_INFO << "Subsribed to "<< m_CameraTopic <<"\n";
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
        return m_OutSize;
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        outFrame = m_ReceivedImage;

        // If there's no error, then we have updated frame and so return true
        return true;
    }


private:
    const std::string m_CameraTopic;
    std::string m_CameraName;
    gazebo::transport::NodePtr m_ImageNode;
    gazebo::transport::SubscriberPtr m_ImageSub;
    cv::Size m_OutSize;
    char *m_Data;
    cv::Mat m_ReceivedImage;
    
    void OnImageMsg(ConstImageStampedPtr &msg)
    {
        // std::cout << msg->image().width() << std::endl;
        // std::cout << msg->image().height() << std::endl;
        // std::cout << msg->image().pixel_format() << std::endl;
        // std::cout << std::endl;
        
        m_OutSize.width = (int) msg->image().width();
        m_OutSize.height = (int) msg->image().height();
        m_Data = new char[msg->image().data().length() + 1];

        memcpy(m_Data, msg->image().data().c_str(), msg->image().data().length());
        m_ReceivedImage = cv::Mat(m_OutSize.height, m_OutSize.width, CV_8UC3, m_Data);

        // cv::imshow("camera", m_ReceivedImage);
        // cv::waitKey(1);
        delete m_Data;  // DO NOT FORGET TO DELETE THIS, 
                    // ELSE GAZEBO WILL TAKE ALL YOUR MEMORY
    }
}; // GazeboCameraInput
} // Video
} // BoBRobotics
