// BoB robotics includes
#include "common/background_exception_catcher.h"
#include "common/logging.h"
#include "video/netsink.h"

// Standard C++ includes
#include <chrono>
#include <stdexcept>

using namespace std::literals;

namespace BoBRobotics {
namespace Video {

NetSink::NetSink(Net::Connection &connection, Input &input)
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

NetSink::NetSink(Net::Connection &connection, const cv::Size &frameSize, const std::string &cameraName)
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

NetSink::~NetSink()
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
void NetSink::sendFrame(const cv::Mat &frame)
{
    // Wait for start acknowledgement
    m_AckSemaphore.waitOnce();

    sendFrameInternal(frame);
}

void NetSink::sendFrameInternal(const cv::Mat &frame)
{
    cv::imencode(".jpg", frame, m_Buffer);

    auto socket = m_Connection.getSocketWriter();
    socket.send("IMG FRAME " + std::to_string(m_Buffer.size()) + "\n");
    socket.send(m_Buffer.data(), m_Buffer.size());
}

void NetSink::onCommandReceived(const Net::Command &command)
{
    if (command[1] != "START") {
        throw Net::BadCommandError();
    }

    // ACK the command and tell client the camera resolution
    m_Connection.getSocketWriter().send("IMG PARAMS " + std::to_string(m_FrameSize.width) + " " +
                                    std::to_string(m_FrameSize.height) + " " +
                                    m_Name + "\n");
}

void NetSink::onCommandReceivedAsync(const Net::Command &command)
{
    // Handle command
    onCommandReceived(command);

    // start thread to transmit images in background
    m_Thread = std::thread(&NetSink::runAsync, this);
}

void NetSink::onCommandReceivedSync(const Net::Command &command)
{
    // Handle command
    onCommandReceived(command);

    // Raise semaphore
    m_AckSemaphore.notify();
}

void NetSink::runAsync()
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

} // Video
} // BoBRobotics
