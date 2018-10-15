/*
 * This header provides various typedefs and includes for networking code to be
 * run on both Windows and *nix. This is necessary because there are small
 * differences in the argument types for the different socket functions and
 * certain macros are defined on one platform and not others etc.
 */

#pragma once

// Headers to be included
#ifdef _WIN32
// Our common header for including windows.h with the right #defines in place
#include "windows_include.h"

// Include the (new) winsock API
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>

// Standard C includes
#include <cwchar>

// Macros defined in Linux but not Windows
#define MSG_NOSIGNAL 0
#define INET_ADDRSTRLEN 22

#pragma comment(lib, "Ws2_32.lib")
#else

// Standard C includes
#include <cerrno>
#include <cstring>

// POSIX includes
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// Macro defined in Windows but not Linux
#define INVALID_SOCKET -1
#endif

// Typedefs for buffers
#ifdef _WIN32
typedef int socklen_t;
typedef int bufflen_t;
typedef char *readbuff_t;
typedef const char *sendbuff_t;
#else
// NB: socklen_t is already defined on *nix
typedef size_t bufflen_t;
typedef void *readbuff_t;
typedef const void *sendbuff_t;
#endif

/*
 * Sockets are different types too. I think I've done this in such a way that
 * it'll compile with mingw (which is *nix-like, so halfway between Windows and
 * *nix).
 */
#ifdef _MSC_VER
typedef SOCKET socket_t;

// Windows provides a closesocket() function instead
inline void
close(socket_t sock)
{
    closesocket(sock);
}
#else
// Sockets are file descriptors in *nix
typedef int socket_t;
#endif

namespace BoBRobotics {
namespace OS {
namespace Net {
/*!
 * \brief A simple wrapper for WSAStartup() and WSACleanup() on Windows
 *
 * This class does nothing on *nix.
 *
 * Use it like so:
 *   int main()
 *   {
 *      try {
 *          BoBRobotics::OS::Net::WindowsNetworking net;
 *          // ... rest of program
 *      } catch (std::exception &e) {
 *          std::cerr << "Uncaught exception: " << e.what() << std::endl;
 *      }
 *   }
 */
class WindowsNetworking
{
public:
#ifdef _WIN32
    WindowsNetworking()
    {
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != NO_ERROR) {
            throw std::runtime_error("Error at WSAStartup");
        }
    }

    ~WindowsNetworking()
    {
        WSACleanup();
    }
#else
    // Empty constructor needed so that variables don't appear to be unused
    WindowsNetworking()
    {}
#endif
}; // WindowsNetworking

#ifdef _WIN32
//! Get the last networking error code
inline int
lastError()
{
    return WSAGetLastError();
}

//! Get the error messgae which corresponds to the given code
inline std::string
errorMessage(int err = lastError())
{
    wchar_t *s = nullptr;
    FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                  nullptr,
                  err,
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPWSTR) &s,
                  0,
                  nullptr);

    // Convert wide chars to regular string
    std::string msg(&s[0], &s[wcslen(s)]);

    // Free memory from heap
    LocalFree(s);

    return msg;
}
#else
inline int
lastError()
{
    return errno;
}

inline std::string
errorMessage(int err = lastError())
{
    return std::strerror(err);
}
#endif
} // Net
} // OS
} // BoBRobotics
