/*
 * This class provides methods for sending and receiving data from a network
 * socket.
 */

#pragma once

// BoB robotics includes
#include "os/net.h"

// Standard C++ includes
#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Net {

//! Represents a command read from the network
using Command = std::vector<std::string>;

//! An exception thrown if the socket is deliberately being closed
class SocketClosedError : public std::runtime_error
{
public:
    SocketClosedError();
};

//! An exception thrown if a command received over the network is badly formed
class BadCommandError : public std::runtime_error
{
public:
    BadCommandError();
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
    Socket(const socket_t handle);

    Socket(int domain, int type, int protocol);

    virtual ~Socket();

    //! Close the socket
    void close();

    //! Check if socket is still open
    bool isOpen() const;

    //! Get the current socket handle this object holds
    socket_t getHandle() const;

    size_t read(void *buffer, const size_t length);

    //! Send a buffer of specified length through the socket
    void send(const void *buffer, size_t length);

    //! Send a string over the socket.
    void send(const std::string &msg);

    // Object is non-copyable
    Socket(const Socket &) = delete;
    void operator=(const Socket &) = delete;
    Socket &operator=(Socket &&) = default;

    Socket(Socket &&old);

private:
    std::atomic<socket_t> m_Handle;

    void throwError(const std::string &msg);
}; // Socket
} // Net
} // BoBRobotics
