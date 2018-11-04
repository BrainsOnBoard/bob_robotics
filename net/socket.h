/*
 * This class provides methods for sending and receiving data from a network
 * socket.
 */

#pragma once

// BoB robotics includes
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
    static const size_t DefaultBufferSize = 1024 * 8; //! Default buffer size, in bytes
    static const int DefaultListenPort = 2000;        //! Default listening port
    static const bool PrintDebug = false;

    Socket(const socket_t handle, bool print = PrintDebug)
      : m_Buffer(DefaultBufferSize)
      , m_Handle(handle)
      , m_Print(print)
    {
        if (!isOpen()) {
            throw OS::Net::NetworkError("Could not initialise socket");
        }
    }

    Socket(int domain, int type, int protocol, bool print = PrintDebug)
      : Socket(socket(domain, type, protocol), print)
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
    socket_t getHandle() const
    {
        return m_Handle;
    }

    //! Read a plaintext command, splitting it into separate words
    Command readCommand()
    {
        std::string line = readLine();
        std::istringstream iss(line);
        Command results(std::istream_iterator<std::string>{ iss },
                        std::istream_iterator<std::string>());
        return results;
    }

    //! Read a specified number of bytes into a buffer
    void read(void *buffer, size_t len)
    {
        // initially, copy over any leftover bytes in m_Buffer
        size_t start = 0;
        if (m_BufferBytes > 0) {
            size_t tocopy = std::min(len, m_BufferBytes);
            memcpy(buffer, &m_Buffer[m_BufferStart], tocopy);
            start += tocopy;
            len -= tocopy;
            debitBytes(tocopy);
        }

        // keep reading from socket until we have enough bytes
        while (len > 0) {
            size_t nbytes = readOnce((char *) buffer, start, len);
            start += nbytes;
            len -= nbytes;
        }
    }

    //! Read a single line in, stopping at a newline char
    std::string readLine()
    {
        std::ostringstream oss;
        while (true) {
            if (m_BufferBytes == 0) {
                m_BufferBytes += readOnce(m_Buffer.data(),
                                          m_BufferStart,
                                          DefaultBufferSize - m_BufferStart);
            }

            // look for newline char
            for (size_t i = 0; i < m_BufferBytes; i++) {
                char &c = m_Buffer[m_BufferStart + i];
                if (c == '\n') {
                    c = '\0';
                    oss << std::string(&m_Buffer[m_BufferStart]);
                    debitBytes(i + 1);

                    std::string outstring = oss.str();
                    if (m_Print) {
                        std::cout << "<<< " << outstring << std::endl;
                    }
                    return outstring;
                }
            }

            // if newline is not present, append the text we received and try
            // another read
            oss << std::string(&m_Buffer[m_BufferStart], m_BufferBytes);
            debitBytes(m_BufferBytes);
        }
    }

    //! Send a buffer of specified length through the socket
    void send(const void *buffer, size_t len)
    {
        const auto ret = ::send(m_Handle, static_cast<sendbuff_t>(buffer), static_cast<bufflen_t>(len), OS::Net::sendFlags);
        if (ret == -1) {
            throwError("Could not send");
        }
    }

    //! Send a string over the socket.
    void send(const std::string &msg)
    {
        send(msg.c_str(), msg.length());

        if (m_Print) {
            std::cout << ">>> " << msg;
        }
    }

    // Object is non-copyable
    Socket(const Socket &) = delete;
    void operator=(const Socket &) = delete;
    Socket &operator=(Socket &&) = default;

    Socket(Socket &&old)
      : m_Buffer(std::move(old.m_Buffer))
      , m_BufferStart(old.m_BufferStart)
      , m_BufferBytes(old.m_BufferBytes)
      , m_Handle(old.m_Handle.load())
      , m_Print(old.m_Print)
    {
        old.m_Handle = INVALID_SOCKET;
    }

private:
    std::vector<char> m_Buffer;
    size_t m_BufferStart = 0;
    size_t m_BufferBytes = 0;
    std::atomic<socket_t> m_Handle;
    bool m_Print;

    // Debit the byte store by specified amount
    void debitBytes(size_t nbytes)
    {
        m_BufferStart += nbytes;
        if (m_BufferStart == DefaultBufferSize) {
            m_BufferStart = 0;
        }
        m_BufferBytes -= nbytes;
    }

    // Make a single call to recv.
    size_t readOnce(char *buffer, size_t start, size_t maxlen)
    {
        const auto len = recv(m_Handle, static_cast<readbuff_t>(&buffer[start]), static_cast<bufflen_t>(maxlen), 0);
        if (len == -1) {
            throwError("Could not read from socket");
        }

        return static_cast<size_t>(len);
    }

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
