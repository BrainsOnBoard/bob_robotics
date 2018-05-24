#pragma once

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>

#define MSG_NOSIGNAL 0
#define INET_ADDRSTRLEN 22

typedef int socklen_t;
typedef int bufflen_t;
typedef char *readbuff_t;
typedef const char *sendbuff_t;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

typedef size_t bufflen_t;
typedef void *readbuff_t;
typedef const void *sendbuff_t;
#define INVALID_SOCKET -1
#endif

#ifdef _MSC_VER
typedef SOCKET socket_t;

inline void
close(socket_t sock)
{
    closesocket(sock);
}

#pragma comment(lib, "Ws2_32.lib")
#else
typedef int socket_t;
#endif

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
