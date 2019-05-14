#pragma once

// BoB robotics includes
#include "os/net.h"

// Standard C++ includes
#include <string>

// Standard C includes
#include <cstdint>

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
    CaptureControl();

    CaptureControl(const std::string &hostname,
                   uint16_t port,
                   const std::string &capturePath);

    ~CaptureControl();

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    bool connect(const std::string &hostname,
                 uint16_t port,
                 const std::string &capturePath);

    bool startRecording(const std::string &recordingName);

    bool stopRecording(const std::string &recordingName);

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::string m_CapturePath;
    int m_Socket;
    sockaddr_in m_RemoteAddress;
    uint32_t m_CapturePacketID;

};
} // namespace Vicon
} // BoBRobotics
