/*
 * This header provides various typedefs and includes for networking code to be
 * run on both Windows and *nix. This is necessary because there are small
 * differences in the argument types for the different socket functions and
 * certain macros are defined on one platform and not others etc.
 */

#pragma once

// Standard C++ includes
#include <string>
#include <stdexcept>

// Headers to be included
#ifdef _WIN32
// Our common header for including windows.h with the right #defines in place
#include "windows_include.h"

// Include the (new) winsock API
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>

// Macro defined in Linux but not Windows
#define INET_ADDRSTRLEN 22

#pragma comment(lib, "Ws2_32.lib")
#else

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
/*
 * We set sendFlags to MSG_NOSIGNAL  on Linux, because otherwise a broken pipe
 * will terminate the program.
 */
#ifdef _WIN32
constexpr int sendFlags = 0;
#else
constexpr int sendFlags = MSG_NOSIGNAL;
#endif

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
    static void initialise();

#ifdef _WIN32
private:
    WindowsNetworking();
    ~WindowsNetworking();
#else
    WindowsNetworking() = delete;
#endif
}; // WindowsNetworking

//! Get the last networking error code
int
lastError();

std::string
errorMessage(int err = lastError());

//! An exception thrown if an error signal is given by network interface
class NetworkError : public std::runtime_error
{
public:
    NetworkError(const std::string &msg);
};
} // Net
} // OS
} // BoBRobotics
