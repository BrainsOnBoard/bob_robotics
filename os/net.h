#pragma once

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2tcpip.h>

#define MSG_NOSIGNAL 0
#define INET_ADDRSTRLEN 22
typedef int socklen_t;

typedef const char buff_t;
typedef char mybuff_t;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

typedef unsigned char buff_t;
typedef unsigned char mybuff_t;

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

// C++ includes
#include <chrono>
#include <thread>

namespace GeNNRobotics {
namespace OS {
namespace Net {
int
readBlocking(socket_t sock, void *buff, size_t bufflen)
{
#ifdef _MSC_VER
    return recv(sock, buff, DefaultBufferSize, 0);
#else
    int len;
    while ((len = read(sock, buff, bufflen)) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return len;
#endif
}
} // Net
} // OS
} // GeNNRobotics
