#pragma once

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

// Standard C includes
#include <cassert>
#include <cstring>

// POSIX includes
#ifdef _WIN32
    #include <winsock2.h>
#else
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <unistd.h>
#endif

// BoB robotics includes
#include "../common/geometry.h"

using namespace units::time;
using namespace units::velocity;
using namespace BoBRobotics::Geometry;

namespace BoBRobotics {
//----------------------------------------------------------------------------
// Vicon Typedefines
//----------------------------------------------------------------------------
namespace Vicon
{
template <class T>
using Vector = T[3];

using Point3 = Vector<meter_t>;
using Attitude = Vector<radian_t>;
using Velocity3 = Vector<meters_per_second_t>;

//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and translation
class ObjectData
{
public:
    ObjectData()
      : m_FrameNumber{ 0 }
      , m_Translation{ 0_m, 0_m, 0_m }
      , m_Rotation{ 0_rad, 0_rad, 0_rad }
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, const Point3 &translation, const Attitude &rotation)
    {
        // Cache frame number
        m_FrameNumber = frameNumber;

        // Copy vectors into class
        std::copy(std::begin(translation), std::end(translation), std::begin(m_Translation));
        std::copy(std::begin(rotation), std::end(rotation), std::begin(m_Rotation));
    }

    uint32_t getFrameNumber() const { return m_FrameNumber; }
    const Point3 &getTranslation() const{ return m_Translation; }
    const Attitude &getRotation() const{ return m_Rotation; }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber;
    Point3 m_Translation;
    Attitude m_Rotation;
};

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
public:
    ObjectDataVelocity() : m_Velocity{0_mps, 0_mps, 0_mps}
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, const Point3 &translation, const Attitude &rotation)
    {
        constexpr second_t frameS = 10_ms;
        constexpr second_t smoothingS = 30_ms;

        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const second_t deltaS = frameS * deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - exp(-deltaS / smoothingS);

        // Calculate instantaneous velocity
        const auto &oldTranslation = getTranslation();
        Velocity3 instVelocity;
        const auto calcVelocity = [deltaS](meter_t curr, meter_t prev) {
            return (curr - prev) / deltaS;
        };
        std::transform(std::begin(translation), std::end(translation),
                       std::begin(oldTranslation), std::begin(instVelocity),
                       calcVelocity);

        // Exponentially smooth velocity
        const auto smoothVelocity = [alpha](meters_per_second_t inst, meters_per_second_t prev) {
            return (alpha * inst) + ((1.0 - alpha) * prev);
        };
        std::transform(std::begin(instVelocity), std::end(instVelocity), std::begin(m_Velocity), std::begin(m_Velocity), smoothVelocity);

        // Superclass
        ObjectData::update(frameNumber, translation, rotation);
    }

    const Velocity3 &getVelocity() const
    {
        return m_Velocity;
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Velocity3 m_Velocity;
};

//----------------------------------------------------------------------------
// Vicon::UDPClient
//----------------------------------------------------------------------------
// Receiver for Vicon UDP streams
template<typename ObjectDataType>
class UDPClient
{
    using UDPClientCallback = void (*)(uint id, const ObjectDataType &data, void *userData);

public:
    UDPClient(){}
    UDPClient(unsigned int port)
    {
        if(!connect(port)) {
            throw std::runtime_error("Cannot connect");
        }
    }

    virtual ~UDPClient()
    {
        // Set quit flag and join read thread
        if(m_ReadThread.joinable()) {
            m_ShouldQuit = true;
            m_ReadThread.join();
        }
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    bool connect(unsigned int port)
    {
        // Create socket
        int socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(socket < 0) {
            std::cerr << "Cannot open socket: " << strerror(errno) << std::endl;
            return false;
        }

        // Set socket to have 1s read timeout
        // **NOTE** this is largely to allow read thread to be stopped
#ifdef _WIN32
        DWORD timeout = 1000;
#else
        timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
#endif
        if(setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
            std::cerr << "Cannot set socket timeout: " << strerror(errno) << std::endl;
            return false;
        }

        // Create socket address structure
        sockaddr_in localAddress;
        memset(&localAddress, 0, sizeof(sockaddr_in));
        localAddress.sin_family = AF_INET;
        localAddress.sin_port = htons(port);
        localAddress.sin_addr.s_addr = htonl(INADDR_ANY);

        // Bind socket to local port
        if(bind(socket, reinterpret_cast<sockaddr*>(&localAddress), sizeof(localAddress)) < 0) {
            std::cerr << "Cannot bind socket: " << strerror(errno) << std::endl;
            return false;
        }

        // Clear atomic stop flag and start thread
        m_ShouldQuit = false;
        m_ReadThread = std::thread(&UDPClient::readThread, this, socket);
        return true;
    }

    unsigned int getNumObjects()
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        return m_ObjectData.size();
    }

    ObjectDataType getObjectData(unsigned int id)
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        if(id < m_ObjectData.size()) {
            return m_ObjectData[id];
        }
        else {
            throw std::runtime_error("Invalid object id:" + std::to_string(id));
        }
    }

    void setReadCallback(UDPClientCallback callback, void *userData)
    {
        m_ReadCallback = callback;
        m_ReadUserData = userData;
    }

private:
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void updateObjectData(unsigned int id, uint32_t frameNumber, const Point3 &translation, const Attitude &rotation)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if(id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }

        // Update object data with translation and rotation
        m_ObjectData[id].update(frameNumber, translation, rotation);

        // Execute callback function, if set. Note that we copy by value here,
        // which is presumably less efficient, but is thread safe
        if (m_ReadCallback) {
            m_ReadCallback(id, m_ObjectData[id], m_ReadUserData);
        }
    }

    void readThread(int socket)
    {
        // Create buffer for reading data
        // **NOTE** this is the maximum size supported by Vicon so will support all payload sizes
        uint8_t buffer[1024];

        // Loop until quit flag is set
        for(unsigned int f = 0; !m_ShouldQuit; f++) {
            // Read datagram
            const ssize_t bytesReceived = recvfrom(socket, &buffer[0], 1024,
                                                   0, NULL, NULL);

            // If there was an error
            if(bytesReceived == -1) {
                // If this was a timeout, continue
                if(errno == EAGAIN || errno == EINTR) {
                    continue;
                }
                // Otherwise, display error and stop
                else {
                    std::cerr << "Cannot read datagram: " << strerror(errno) << std::endl;
                    break;
                }
            }
            // Otherwise, if data was received
            else {
                // Read frame number
                uint32_t frameNumber;
                memcpy(&frameNumber, &buffer[0], sizeof(uint32_t));

                // Read items in block
                const unsigned int itemsInBlock = (unsigned int)buffer[4];

                // Loop through items in blcok
                unsigned int itemOffset = 5;
                for(unsigned int i = 0; i < itemsInBlock; i++) {
                    // Read object ID
                    const unsigned int objectID = (unsigned int)buffer[itemOffset];

                    // Read size of item
                    uint16_t itemDataSize;
                    memcpy(&itemDataSize, &buffer[itemOffset + 1], sizeof(uint16_t));
                    assert(itemDataSize == 72);

                    // Read object translation + convert to metres
                    Vector<millimeter_t> translationMM;
                    memcpy(&translationMM[0], &buffer[itemOffset + 27], 3 * sizeof(double));
                    Point3 translation {translationMM[0], translationMM[1], translationMM[2]};

                    // Read object rotation
                    Attitude rotation;
                    memcpy(&rotation[0], &buffer[itemOffset + 51], 3 * sizeof(double));

                    // Update item
                    updateObjectData(objectID, frameNumber, translation, rotation);

                    // Update offset for next offet
                    itemOffset += itemDataSize;
                }
            }
        }

        // Close socket
        close(socket);
    }

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::atomic<bool> m_ShouldQuit;
    std::thread m_ReadThread;
    UDPClientCallback m_ReadCallback = nullptr;
    void *m_ReadUserData;

    std::mutex m_ObjectDataMutex;
    std::vector<ObjectDataType> m_ObjectData;
};
} // namespace Vicon
} // BoBRobotics
