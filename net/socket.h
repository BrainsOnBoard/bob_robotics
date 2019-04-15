/*
 * This class provides methods for sending and receiving data from a network
 * socket.
 */

#pragma once

// BoB robotics includes
#include "../common/logging.h"
#include "../os/net.h"

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace BoBRobotics {
namespace Net {

//! Represents a command read from the network
using Command = std::vector<std::string>;

//! An exception thrown if the socket is deliberately being closed
class SocketClosedError : public std::runtime_error
{
public:
    SocketClosedError() : std::runtime_error("Socket is closed")
    {}
};

//! An exception thrown if a command received over the network is badly formed
class BadCommandError : public std::runtime_error
{
public:
    BadCommandError()
      : std::runtime_error("Bad command received")
    {}
};

//----------------------------------------------------------------------------
// BoBRobotics::Net::Server
//----------------------------------------------------------------------------
/*!
 * \brief A class for operating on sockets in a platform-independent way
 *
 * In Windows, this class represents an underlying SOCKET pointer and in *nix,
 * it represents a socket's file handle. This class provides convenience methods
 * for sending and receiving data from the socket.
 *
 * A typical (plaintext) command sent over a socket looks like this:
 *     TNK 0.5 0.5
 * where the first word indicates the command name and the other words are parameters.
 */
class Socket
{
public:
    Socket(const socket_t handle)
      : m_Handle(handle)
    {
        if (!isOpen()) {
            throw OS::Net::NetworkError("Could not initialise socket");
        }
    }

    Socket(int domain, int type, int protocol)
      : Socket(socket(domain, type, protocol))
    {}

    virtual ~Socket()
    {
        close();
    }

    //! Close the socket
    void close()
    {
        const socket_t handle = m_Handle.exchange(INVALID_SOCKET);
        if (handle != INVALID_SOCKET) {
            ::close(handle);
        }
    }

    //! Check if socket is still open
    bool isOpen() const { return m_Handle != INVALID_SOCKET; }

    //! Get the current socket handle this object holds
    socket_t getHandle() const { return m_Handle; }

    size_t read(void *buffer, const size_t length)
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

    //! Send a buffer of specified length through the socket
    void send(const void *buffer, size_t length)
    {
        const auto ret = ::send(m_Handle,
                                reinterpret_cast<sendbuff_t>(buffer),
                                static_cast<bufflen_t>(length),
                                OS::Net::sendFlags);
        if (ret == -1) {
            throwError("Could not send");
        }
    }

    //! Send a string over the socket.
    void send(const std::string &msg)
    {
        send(msg.c_str(), msg.size());
        LOG_VERBOSE << ">>> " << msg;
    }

    // Object is non-copyable
    Socket(const Socket &) = delete;
    void operator=(const Socket &) = delete;
    Socket &operator=(Socket &&) = default;

    Socket(Socket &&old)
      : m_Handle(old.m_Handle.load())
    {
        old.m_Handle = INVALID_SOCKET;
    }

private:
    std::atomic<socket_t> m_Handle;

    void throwError(const std::string &msg)
    {
        if (isOpen()) {
            close();
            throw OS::Net::NetworkError(msg);
        } else {
            throw SocketClosedError();
        }
    }
}; // Socket
} // Net
} // BoBRobotics
