// BoB robotics includes
#include "os/net.h"

#ifndef _WIN32
// Standard C includes
#include <cerrno>
#include <cstring>
#endif // !_WIN32

namespace BoBRobotics {
namespace OS {
namespace Net {
#ifdef _WIN32
void
WindowsNetworking::initialise()
{
    static WindowsNetworking net;
}

WindowsNetworking::WindowsNetworking()
{
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != NO_ERROR) {
        throw std::runtime_error("Error at WSAStartup");
    }
}

WindowsNetworking::~WindowsNetworking()
{
    WSACleanup();
}

int
lastError()
{
    return WSAGetLastError();
}

std::string
errorMessage(int err)
{
    wchar_t *s = nullptr;
    FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
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
void
WindowsNetworking::initialise()
{}

int
lastError()
{
    return errno;
}

std::string
errorMessage(int err)
{
    return std::strerror(err);
}
#endif

NetworkError::NetworkError(const std::string &msg)
    : std::runtime_error(msg + " (" + std::to_string(OS::Net::lastError()) + ": " + OS::Net::errorMessage() + ")")
{}

} // Net
} // OS
} // BoBRobotics
