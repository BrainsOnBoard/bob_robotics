#pragma once

// BoB robotics includes
#include "connection.h"

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Net {
//----------------------------------------------------------------------------
// BoBRobotics::Net::Client
//----------------------------------------------------------------------------
/*!
 * \brief General-purpose TCP client
 *
 * To be used with corresponding Server object. Various sink/source-type
 * objects are used for either sending or receiving data from the server.
 */
class Client
  : public Connection
{
public:
    //! Create client and connect to host over TCP
    Client(const std::string &host = getDefaultIP(),
           uint16_t port = DefaultListenPort);

    const std::string &getIP() const;
    static std::string getDefaultIP();

private:
    const std::string m_IP;
}; // Client
} // Net
} // BoBRobotics
