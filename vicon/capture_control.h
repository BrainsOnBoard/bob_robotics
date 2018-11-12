#pragma once

// BoB robotics includes
#include "../common/assert.h"

// Standard C++ includes
#include <string>
#include <sstream>

// Standard C includes
#include <cstdint>
#include <cstring>
#include <ctime>

// POSIX networking includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace BoBRobotics {
namespace Vicon
{
//----------------------------------------------------------------------------
// BoBRobotics::Vicon::CaptureControl
//----------------------------------------------------------------------------
//! Transmitter for sending capture control packets to the Vicon motion-tracking system
class CaptureControl
{

public:
    CaptureControl() : m_Socket(-1){}
    CaptureControl(const std::string &hostname, uint16_t port,
                   const std::string &capturePath)
    {
        if(!connect(hostname, port, capturePath)) {
            throw std::runtime_error("Cannot connect to to Vicon Tracker");
        }
    }
    ~CaptureControl()
    {
         if(m_Socket >= 0) {
             close(m_Socket);
         }
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    bool connect(const std::string &hostname, uint16_t port,
                 const std::string &capturePath)
    {
        // Stash capture path
        m_CapturePath = capturePath;

        // Create socket
        m_Socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(m_Socket < 0) {
            std::cerr << "Cannot open socket: " << strerror(errno) << std::endl;
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

    bool startRecording(const std::string &recordingName)
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
            std::cerr << "Cannot send start message:" << strerror(errno) << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

    bool stopRecording(const std::string &recordingName)
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
            std::cerr << "Cannot send stop message:" << strerror(errno) << std::endl;
            return false;
        }
        else {
            return true;
        }
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    int m_Socket;

    std::string m_CapturePath;
    uint32_t m_CapturePacketID;

    sockaddr_in m_RemoteAddress;
};
} // namespace Vicon
} // BoBRobotics
