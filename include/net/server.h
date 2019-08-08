#pragma once

// BoB robotics includes
#include "connection.h"
#include "socket.h"

// Standard C includes
#include <cstdint>

namespace BoBRobotics {
namespace Net {
//----------------------------------------------------------------------------
// BoBRobotics::Net::Server
//----------------------------------------------------------------------------
/*!
 * \brief A general-purpose TCP server
 *
 * To be used with corresponding Client object. Various sink/source-type
 * objects are used for either sending or receiving data to the client.
 */
class Server
{
public:
    //! Create a new server, listening on the specified port
    Server(uint16_t port = Connection::DefaultListenPort);

    Connection waitForConnection() const;

private:
    const Socket m_ListenSocket;
};
} // Net
} // BoBRobotics
