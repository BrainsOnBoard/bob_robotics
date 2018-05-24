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
    static const size_t DefaultBufferSize = 512;
    static const int DefaultListenPort = 2000;
    static const bool PrintDebug = false;

    Socket(bool print = PrintDebug)
      : m_Print(print)
    {}

    Socket(socket_t sock, bool print = PrintDebug)
      : m_Print(print)
    {
        setSocket(sock);
    }

    virtual ~Socket()
    {
        if (m_Socket != INVALID_SOCKET) {
            close(m_Socket);
        }
    }

    socket_t getSocket() const
    {
        return m_Socket;
    }

    std::vector<std::string> readCommand()
    {
        std::string line = readLine();
        std::istringstream iss(line);
        std::vector<std::string> results(
                std::istream_iterator<std::string>{ iss },
                std::istream_iterator<std::string>());
        return results;
    }

    void read(void *buffer, size_t len)
    {
        std::lock_guard<std::mutex> guard(m_ReadMutex);
        checkSocket();

        size_t start = 0;
        // initially, copy over any leftover bytes in m_Buffer
        if (m_BufferBytes > 0) {
            size_t tocopy = std::min(len, m_BufferBytes);
            memcpy(buffer, &m_Buffer[m_BufferStart], tocopy);
            start += tocopy;
            len -= tocopy;
            debitBytes(tocopy);
        }

        while (len > 0) {
            size_t nbytes = readOnce((char *) buffer, start, len);
            start += nbytes;
            len -= nbytes;
        }
    }

    /*
     * Read a single line in. This function's probably not terribly robust.
     */
    std::string readLine()
    {
        std::lock_guard<std::mutex> guard(m_ReadMutex);
        checkSocket();

        std::ostringstream oss;
        while (true) {
            if (m_BufferBytes == 0) {
                m_BufferBytes += readOnce(m_Buffer,
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

    void send(const void *buffer, size_t len)
    {
        std::lock_guard<std::mutex> guard(m_SendMutex);
        checkSocket();

        int ret = ::send(m_Socket, (sendbuff_t) buffer, (bufflen_t) len, MSG_NOSIGNAL);
        if (ret == -1) {
            throw socket_error("Could not send " + errorMessage());
        }
    }

    void send(const std::string &msg)
    {
        send(msg.c_str(), msg.length());

        if (m_Print) {
            std::cout << ">>> " << msg;
        }
    }

    void setSocket(socket_t sock)
    {
        std::lock_guard<std::mutex> guard(m_ReadMutex);

        m_Socket = sock;
        checkSocket();
    }

private:
    char m_Buffer[DefaultBufferSize];
    size_t m_BufferStart = 0;
    size_t m_BufferBytes = 0;
    std::mutex m_ReadMutex, m_SendMutex;
    bool m_Print;
    socket_t m_Socket = INVALID_SOCKET;

    void debitBytes(size_t nbytes)
    {
        m_BufferStart += nbytes;
        if (m_BufferStart == DefaultBufferSize) {
            m_BufferStart = 0;
        }
        m_BufferBytes -= nbytes;
    }

    void checkSocket()
    {
        if (m_Socket == INVALID_SOCKET) {
            throw std::runtime_error("Bad socket " + errorMessage());
        }
    }

    size_t readOnce(char *buffer, size_t start, size_t maxlen)
    {
#ifdef _MSC_VER
        int len = recv(m_Socket, (readbuff_t) &buffer[start], (bufflen_t) maxlen, 0);
#else
        int len;
        while ((len = read(m_Socket, (readbuff_t) &buffer[start], (bufflen_t) maxlen)) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
#endif
        if (len == -1) {
            throw socket_error("Could not read from socket " + errorMessage());
        }

        return (size_t) len;
    }

    static std::string errorMessage()
    {
        return " (" + std::to_string(errno) + ": " + std::strerror(errno) + ")";
    }
}; // Socket
} // Net
} // GeNNRobotics
