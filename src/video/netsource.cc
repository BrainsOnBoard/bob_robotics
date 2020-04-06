// BoB robotics includes
#include "video/netsource.h"

namespace BoBRobotics {
namespace Video {

NetSource::NetSource(Net::Connection &connection)
  : m_Connection(connection)
{
    // Handle incoming IMG commands
    connection.setCommandHandler("IMG", [this](Net::Connection &connection, const Net::Command &command) {
        onCommandReceived(connection, command);
    });

    // When connected, send command to start streaming
    connection.getSocketWriter().send("IMG START\n");
}

NetSource::~NetSource()
{
    // Ignore IMG commands
    m_Connection.setCommandHandler("IMG", nullptr);
}

std::string
NetSource::getCameraName() const
{
    m_ParamsSemaphore.waitOnce();
    return m_CameraName;
}

cv::Size
NetSource::getOutputSize() const
{
    m_ParamsSemaphore.waitOnce();
    return m_CameraResolution;
}

bool
NetSource::needsUnwrapping() const
{
    m_ParamsSemaphore.waitOnce();
    return Input::needsUnwrapping();
}

bool
NetSource::readFrame(cv::Mat &frame)
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

void
NetSource::onCommandReceived(Net::Connection &connection, const Net::Command &command)
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

} // Video
} // BoBRobotics
