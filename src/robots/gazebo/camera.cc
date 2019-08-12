// BoB robotics includes
#include "robots/gazebo/camera.h"
#include "robots/gazebo/node.h"
#include "common/logging.h"
#include "common/macros.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Robots {
namespace Gazebo {

Camera::Camera(const std::string &topic, const bool isPanoramic)
  : Camera(getNode(), topic, isPanoramic)
{}

Camera::Camera(gazebo::transport::NodePtr node, const std::string &topic,
               const bool isPanoramic)
  : m_ImageNode(node)
  , m_IsPanoramic(isPanoramic)
{
    // Subscribe to the topic, and register a callback
    m_ImageSub = m_ImageNode->Subscribe(topic, &Camera::onImageMsg, this);
    LOG_INFO << "Subsribed to " << topic;
}

std::string
Camera::getCameraName() const
{
    return m_IsPanoramic ? "gazebo_panoramic_camera" : "gazebo_camera";
}

cv::Size
Camera::getOutputSize() const
{
    m_HaveReceivedFrameSemaphore.waitOnce();
    return m_ReceivedImage.size();
}

bool
Camera::readFrame(cv::Mat &outFrame)
{
    if (!m_HaveReceivedFrames.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lck(m_Mtx);
    m_ReceivedImage.copyTo(outFrame);

    // If there's no error, then we have updated frame and so return true
    return true;
}

bool
Camera::needsUnwrapping() const
{
    return m_IsPanoramic;
}

void
Camera::onImageMsg(ConstImageStampedPtr &msg)
{
    std::lock_guard<std::mutex> lck(m_Mtx);
    m_ReceivedImage.create(msg->image().height(), msg->image().width(), CV_8UC3);
    std::copy_n(msg->image().data().c_str(),  msg->image().data().length(),
                m_ReceivedImage.data);
    if (!m_HaveReceivedFrames) {
        m_HaveReceivedFrames = true;
        m_HaveReceivedFrameSemaphore.notify();
    }
}

} // Gazebo
} // Robots
} // BoBRobotics
