/*
 * This header provides various typedefs and includes for networking code to be
 * run on both Windows and *nix. This is necessary because there are small
 * differences in the argument types for the different socket functions and
 * certain macros are defined on one platform and not others etc.
 */

#pragma once

// Headers to be included
#ifdef _WIN32
// Our common header for including windows.h with the right macros set
#include "windows_include.h"

// Include the (new) winsock API
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>

// Macros defined in Linux but not Windows
#define MSG_NOSIGNAL 0
#define INET_ADDRSTRLEN 22
#else
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

#pragma comment(lib, "Ws2_32.lib")
#else
// Sockets are file descriptors in *nix
typedef int socket_t;
#endif

// Macros for enabling networking functionality on Windows and cleaning up
#ifdef _WIN32
#define WSAStartup()                                                           \
    {                                                                          \
        WSADATA wsaData;                                                       \
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);                     \
        if (result != NO_ERROR) {                                              \
            throw std::runtime_error("Error at WSAStartup");                   \
        }                                                                      \
    }
#else
#define WSAStartup()                                                           \
    {}
#define WSACleanup()                                                           \
    {}
#endif
