#ifndef _WIN32
// BoB robotics includes
#include "common/macros.h"
#include "common/logging.h"
#include "vicon/capture_control.h"

// Standard C++ includes
#include <sstream>

// Standard C includes
#include <cstring>

// POSIX networking includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace BoBRobotics {
namespace Vicon {

CaptureControl::CaptureControl() : m_Socket(-1){}

CaptureControl::CaptureControl(const std::string &hostname, uint16_t port,
                const std::string &capturePath)
{
    if(!connect(hostname, port, capturePath)) {
        throw std::runtime_error("Cannot connect to to Vicon Tracker");
    }
}

CaptureControl::~CaptureControl()
{
        if(m_Socket >= 0) {
            close(m_Socket);
        }
}

//----------------------------------------------------------------------------
// Public API
//----------------------------------------------------------------------------
bool CaptureControl::connect(const std::string &hostname, uint16_t port,
                const std::string &capturePath)
{
    // Stash capture path
    m_CapturePath = capturePath;

    // Create socket
    m_Socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(m_Socket < 0) {
        LOG_ERROR << "Cannot open socket: " << strerror(errno);
        return false;
    }

        // Create socket address structure
    memset(&m_RemoteAddress, 0, sizeof(sockaddr_in));
    m_RemoteAddress.sin_family = AF_INET,
    m_RemoteAddress.sin_port = htons(port),
    m_RemoteAddress.sin_addr.s_addr = inet_addr(hostname.c_str());

    // Get initial capture packet id from time
    // **NOTE** we intentionally cast this to 32-bit as (supposedly)
    // Vicon Tracker struggles with large values...
    m_CapturePacketID = (uint32_t)time(nullptr);
    return true;
}

bool CaptureControl::startRecording(const std::string &recordingName)
{
    // Create message
    std::stringstream message;
    message << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << std::endl;
    message << "<CaptureStart>" << std::endl;
    message << "<Name VALUE=\"" << recordingName << "\"/>" << std::endl;
    message << "<DatabasePath VALUE=\"" << m_CapturePath << "\"/>" << std::endl;
    message << "<PacketID VALUE=\"" << m_CapturePacketID++ << "\"/>" << std::endl;
    message << "</CaptureStart>" << std::endl;

    // Send message  to tracker
    std::string messageString = message.str();
    if(::sendto(m_Socket, messageString.c_str(), messageString.length(), 0,
                reinterpret_cast<sockaddr*>(&m_RemoteAddress), sizeof(sockaddr_in)) < 0)
    {
        LOG_ERROR << "Cannot send start message:" << strerror(errno);
        return false;
    }
    else {
        return true;
    }
}

bool CaptureControl::stopRecording(const std::string &recordingName)
{
    // Create message
    std::stringstream message;
    message << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << std::endl;
    message << "<CaptureStop>" << std::endl;
    message << "<Name VALUE=\"" << recordingName << "\"/>" << std::endl;
    message << "<DatabasePath VALUE=\"" << m_CapturePath << "\"/>" << std::endl;
    message << "<PacketID VALUE=\"" << m_CapturePacketID++ << "\"/>" << std::endl;
    message << "</CaptureStop>" << std::endl;

    // Send message  to tracker
    std::string messageString = message.str();
    if(::sendto(m_Socket, messageString.c_str(), messageString.length(), 0,
                reinterpret_cast<sockaddr*>(&m_RemoteAddress), sizeof(sockaddr_in)) < 0)
    {
        LOG_WARNING << "Cannot send stop message:" << strerror(errno);
        return false;
    }
    else {
        return true;
    }
}

} // Vicon
} // BoBRobotics
#endif
