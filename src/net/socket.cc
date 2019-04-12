// BoB robotics includes
#include "common/logging.h"
#include "net/socket.h"

namespace BoBRobotics {
namespace Net {

SocketClosedError::SocketClosedError()
  : std::runtime_error("Socket is closed")
{}

BadCommandError::BadCommandError()
  : std::runtime_error("Bad command received")
{}


Socket::Socket(const socket_t handle)
    : m_Handle(handle)
{
    if (!isOpen()) {
        throw OS::Net::NetworkError("Could not initialise socket");
    }
}

Socket::Socket(int domain, int type, int protocol)
    : Socket(socket(domain, type, protocol))
{}

Socket::~Socket()
{
    close();
}

void Socket::close()
{
    const socket_t handle = m_Handle.exchange(INVALID_SOCKET);
    if (handle != INVALID_SOCKET) {
        ::close(handle);
    }
}

bool Socket::isOpen() const { return m_Handle != INVALID_SOCKET; }

socket_t Socket::getHandle() const { return m_Handle; }

size_t Socket::read(void *buffer, const size_t length)
{
    const auto nbytes = recv(m_Handle,
                             reinterpret_cast<readbuff_t>(buffer),
                             static_cast<bufflen_t>(length),
                             0);
    if (nbytes == -1) {
        throwError("Could not read from socket");
    }

    return static_cast<size_t>(nbytes);
}

void Socket::send(const void *buffer, size_t length)
{
    const auto ret = ::send(m_Handle,
                            reinterpret_cast<sendbuff_t>(buffer),
                            static_cast<bufflen_t>(length),
                            OS::Net::sendFlags);
    if (ret == -1) {
        throwError("Could not send");
    }
}

void Socket::send(const std::string &msg)
{
    send(msg.c_str(), msg.size());
    LOG_VERBOSE << ">>> " << msg;
}

Socket::Socket(Socket &&old)
    : m_Handle(old.m_Handle.load())
{
    old.m_Handle = INVALID_SOCKET;
}

void Socket::throwError(const std::string &msg)
{
    if (isOpen()) {
        close();
        throw OS::Net::NetworkError(msg);
    } else {
        throw SocketClosedError();
    }
}
} // Net
} // BoBRobotics
