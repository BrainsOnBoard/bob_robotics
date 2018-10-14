/*
 * This class provides methods for sending and receiving data from a network
 * socket.
 */

#pragma once

// Standard C includes
#include <cerrno>
#include <cstring>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// BoB robotics includes
#include "../os/net.h"

namespace BoBRobotics {
namespace Net {

//! Represents a command read from the network
using Command = std::vector<std::string>;

//! An exception thrown if an error signal is given by a Socket
class SocketError : public std::runtime_error
{
public:
    SocketError(std::string msg)
      : std::runtime_error(msg + " (" + std::to_string(errno) + ": " + std::strerror(errno) + ")")
    {}
};

//! An exception thrown if a command received over the network is badly formed
class BadCommandError : public SocketError
{
public:
    BadCommandError()
      : SocketError("Bad command received")
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
    static const int DefaultListenPort = 2000; //! Default listening port
    static const bool PrintDebug = false;

    /*!
     * \brief Initialise class without a Socket set
     *
     * The Socket can be set later with setSocket().
     */
    Socket(bool print = PrintDebug)
      : m_Buffer(DefaultBufferSize)
      , m_Print(print)
    {}

    //! Initialise class with specified socket
    Socket(socket_t sock, bool print = PrintDebug)
      : Socket(print)
    {
        setSocket(sock);
    }

    //! Close the socket
    virtual ~Socket()
    {
        if (m_Socket != INVALID_SOCKET) {
            close(m_Socket);
        }
    }

    //! Get the current socket handle this object holds
    socket_t getSocket() const
    {
        return m_Socket;
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
        std::lock_guard<std::mutex> guard(m_ReadMutex);
        checkSocket();

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
        std::lock_guard<std::mutex> guard(m_ReadMutex);
        checkSocket();

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
        std::lock_guard<std::mutex> guard(m_SendMutex);
        checkSocket();

        int ret = ::send(m_Socket, static_cast<sendbuff_t>(buffer), static_cast<bufflen_t>(len), MSG_NOSIGNAL);
        if (ret == -1) {
            throw SocketError("Could not send");
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

    //! Set the current socket handle for this connection
    void setSocket(socket_t sock)
    {
        std::lock_guard<std::mutex> guard(m_ReadMutex);

        m_Socket = sock;
        checkSocket();
    }

private:
    std::vector<char> m_Buffer;
    size_t m_BufferStart = 0;
    size_t m_BufferBytes = 0;
    std::mutex m_ReadMutex, m_SendMutex;
    bool m_Print;
    socket_t m_Socket = INVALID_SOCKET;

    /*
     * Debit the byte store by specified amount.
     */
    void debitBytes(size_t nbytes)
    {
        m_BufferStart += nbytes;
        if (m_BufferStart == DefaultBufferSize) {
            m_BufferStart = 0;
        }
        m_BufferBytes -= nbytes;
    }

    /*
     * Check that the current socket is valid.
     */
    void checkSocket()
    {
        if (m_Socket == INVALID_SOCKET) {
            throw SocketError("Bad socket");
        }
    }

    /*
     * Make a single call to read/recv.
     */
    size_t readOnce(char *buffer, size_t start, size_t maxlen)
    {
        int len = recv(m_Socket, static_cast<readbuff_t>(&buffer[start]), static_cast<bufflen_t>(maxlen), 0);
        if (len == -1) {
            throw SocketError("Could not read from socket");
        }

        return static_cast<size_t>(len);
    }
}; // Socket
} // Net
} // BoBRobotics
