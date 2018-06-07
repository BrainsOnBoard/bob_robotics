/*
 * This class provides methods for sending and receiving data from a network
 * socket.
 */

#pragma once

// C includes
#include <cerrno>
#include <cstring>

// C++ includes
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

// GeNN robotics includes
#include "../os/net.h"

namespace GeNNRobotics {
namespace Net {
using Command = std::vector<std::string>;    
    
class socket_error : public std::runtime_error
{
public:
    socket_error(std::string msg)
      : std::runtime_error("Socket error: " + msg)
    {}
};

class bad_command_error : public socket_error
{
public:
    bad_command_error()
      : socket_error("Bad command received")
    {}
};

class Socket
{
public:
    static const size_t DefaultBufferSize = 1024*8;
    static const int DefaultListenPort = 2000;
    static const bool PrintDebug = false;

    /*
     * Initialise class without m_Socket set. It can be set later with setSocket().
     */
    Socket(bool print = PrintDebug)
      : m_Buffer(DefaultBufferSize), m_Print(print)
    {
        m_Buffer.resize(DefaultBufferSize);
    }

    /*
     * Initialise class with specified socket.
     */
    Socket(socket_t sock, bool print = PrintDebug)
      : Socket(print)
    {
        setSocket(sock);
    }

    virtual ~Socket()
    {
        if (m_Socket != INVALID_SOCKET) {
            close(m_Socket);
        }
    }

    /*
     * Get the current socket handle this object holds.
     */
    socket_t getSocket() const
    {
        return m_Socket;
    }

    /*
     * Read a plaintext command, splitting it into separate words.
     */
    Command readCommand()
    {
        std::string line = readLine();
        std::istringstream iss(line);
        Command results(
                std::istream_iterator<std::string>{ iss },
                std::istream_iterator<std::string>());
        return results;
    }

    /*
     * Read a specified number of bytes into a buffer.
     */
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

    /*
     * Read a single line in, stopping at a newline char.
     */
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

    /*
     * Send a buffer of specified length through the socket.
     */
    void send(const void *buffer, size_t len)
    {
        std::lock_guard<std::mutex> guard(m_SendMutex);
        checkSocket();

        int ret = ::send(m_Socket, (sendbuff_t) buffer, (bufflen_t) len, MSG_NOSIGNAL);
        if (ret == -1) {
            throw socket_error("Could not send " + errorMessage());
        }
    }

    /*
     * Send a string over the socket.
     */
    void send(const std::string &msg)
    {
        send(msg.c_str(), msg.length());

        if (m_Print) {
            std::cout << ">>> " << msg;
        }
    }

    /*
     * Set the current socket;
     */
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
            throw socket_error("Bad socket " + errorMessage());
        }
    }

    /*
     * Make a single call to read/recv.
     */
    size_t readOnce(char *buffer, size_t start, size_t maxlen)
    {
#ifdef _MSC_VER
        int len = recv(m_Socket, (readbuff_t) &buffer[start], (bufflen_t) maxlen, 0);
#else
        int len;
        while ((len = ::read(m_Socket, (readbuff_t) &buffer[start], (bufflen_t) maxlen)) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
#endif
        if (len == -1) {
            throw socket_error("Could not read from socket " + errorMessage());
        }

        return (size_t) len;
    }

    /*
     * Get the last error message.
     */
    static std::string errorMessage()
    {
        return " (" + std::to_string(errno) + ": " + std::strerror(errno) + ")";
    }
}; // Socket
} // Net
} // GeNNRobotics
