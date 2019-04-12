// BoB robotics includes
#include "net/client.h"
#include "common/logging.h"
#include "os/net.h"

// Standard C includes
#include <cstdlib>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Net {

Client::Client(const std::string &host,
               uint16_t port)
  : Connection(AF_INET, SOCK_STREAM, 0)
  , m_IP(host)
{
    // Create socket address structure
    in_addr addr;
    addr.s_addr = inet_addr(host.c_str());
    sockaddr_in destAddress;
    destAddress.sin_family = AF_INET;
    destAddress.sin_port = htons(port);
    destAddress.sin_addr = addr;

    // Connect socket
    if (connect(getSocket().getHandle(),
                reinterpret_cast<sockaddr *>(&destAddress),
                sizeof(destAddress)) < 0) {
        getSocket().close();
        throw OS::Net::NetworkError("Cannot connect socket to " + host + ":" +
                                    std::to_string(port));
    }

    LOG_INFO << "Opened socket";
}

const std::string &
Client::getIP() const
{
    return m_IP;
}

std::string
Client::getDefaultIP()
{
    constexpr const char *envVar = "ROBOT_IP", *defaultIP = "127.0.0.1";

    const char *ip = std::getenv(envVar);
    if (!ip) {
        LOG_WARNING << "Environment variable " << envVar
                    << " not set; using " << defaultIP;
        ip = defaultIP;
    }
    return ip;
}

} // Net
} // BoBRobotics
