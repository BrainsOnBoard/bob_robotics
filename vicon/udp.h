#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/pose.h"
#include "../common/stopwatch.h"
#include "../os/net.h"

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace BoBRobotics
{
namespace Vicon
{
using namespace units::literals;

//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and attitude
class ObjectData
  : public Pose3<units::length::millimeter_t, units::angle::radian_t>
{
    using radian_t = units::angle::radian_t;
    using millimeter_t = units::length::millimeter_t;

public:
    ObjectData()
      : m_FrameNumber{ 0 }
    {}

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, std::string &&objectName,
                millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        // Cache frame number
        m_FrameNumber = frameNumber;

        // Update object name
        m_ObjectName = objectName;

        // Copy vectors into class
        position() = { x, y, z };
        attitude() = { yaw, pitch, roll };
    }

    auto getElapsedTime() const
    {
        return m_ElapsedTime;
    }

    void setElapsedTime(const Stopwatch::Duration elapsed)
    {
        m_ElapsedTime = elapsed;
    }

    uint32_t getFrameNumber() const
    {
        return m_FrameNumber;
    }

    const std::string &getObjectName() const
    {
        return m_ObjectName;
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber;
    std::string m_ObjectName;
    Stopwatch::Duration m_ElapsedTime;
};

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
    using radian_t = units::angle::radian_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;
    using millisecond_t = units::time::millisecond_t;

public:
    ObjectDataVelocity() : m_Velocity{0_mps, 0_mps, 0_mps}
    {}

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, std::string &&objectName,
                millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        const Position3<millimeter_t> newPosition{ x, y, z };
        constexpr millisecond_t frameS = 10_ms;
        constexpr millisecond_t smoothingS = 30_ms;

        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const auto deltaS = frameS * deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - units::math::exp(-deltaS / smoothingS);

        // Calculate instantaneous velocity
        Vector3<meters_per_second_t> instVelocity;
        const auto calcVelocity = [deltaS](auto curr, auto prev) {
            return (curr - prev) / deltaS;
        };
        std::transform(std::begin(newPosition), std::end(newPosition),
                       std::begin(position()), std::begin(instVelocity),
                       calcVelocity);

        // Exponentially smooth velocity
        const auto smoothVelocity = [alpha](auto inst, auto prev) {
            return (alpha * inst) + ((1.0 - alpha) * prev);
        };
        std::transform(std::begin(instVelocity), std::end(instVelocity),
                       std::begin(m_Velocity), std::begin(m_Velocity),
                       smoothVelocity);

        // Superclass
        ObjectData::update(frameNumber, std::move(objectName), x, y, z, yaw, pitch, roll);
    }

    const auto &velocity() const
    {
        return m_Velocity;
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Vector3<meters_per_second_t> m_Velocity;
};

//----------------------------------------------------------------------------
// BoBRobotics::Vicon::UDPClient
//----------------------------------------------------------------------------
//! Receiver for Vicon UDP streams
template<typename ObjectDataType = ObjectData>
class UDPClient
{
public:
    class Object
    {
        using millimeter_t = units::length::millimeter_t;
        using radian_t = units::angle::radian_t;

    public:
        Object(UDPClient<ObjectDataType> &client, const unsigned id)
          : m_Client(client)
          , m_Id(id)
        {}

        auto position() const { return getData().position(); }
        auto attitude() const { return getData().attitude(); }
        auto pose() const { return getData(); }

        ObjectDataType getData() const
        {
            return m_Client.getObjectData(m_Id);
        }

    private:
        UDPClient<ObjectDataType> &m_Client;
        const unsigned m_Id;
    };

    UDPClient(){}
    UDPClient(uint16_t port)
    {
        connect(port);
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
    void connect(uint16_t port)
    {
        // Create socket
        int socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(socket < 0) {
            throw OS::Net::NetworkError("Cannot open socket");
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
            throw OS::Net::NetworkError("Cannot set socket timeout");
        }

        // Create socket address structure
        sockaddr_in localAddress;
        memset(&localAddress, 0, sizeof(sockaddr_in));
        localAddress.sin_family = AF_INET;
        localAddress.sin_port = htons(port);
        localAddress.sin_addr.s_addr = htonl(INADDR_ANY);

        // Bind socket to local port
        if(bind(socket, reinterpret_cast<sockaddr*>(&localAddress), sizeof(localAddress)) < 0) {
            throw OS::Net::NetworkError("Cannot bind socket");
        }

        // Clear atomic stop flag and start thread
        m_ShouldQuit = false;
        m_ReadThread = std::thread(&UDPClient::readThread, this, socket);
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
            auto data = m_ObjectData[id].first;
            data.setElapsedTime(m_ObjectData[id].second.elapsed());
            return data;
        }
        else {
            throw std::runtime_error("Invalid object id: " + std::to_string(id));
        }
    }

    Object getObject(unsigned int id)
    {
        return Object(*this, id);
    }

private:
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void updateObjectData(unsigned int id, uint32_t frameNumber,
                          std::string &&objectName,
                          const Vector3<double> &position,
                          const Vector3<double> &attitude)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if (id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }
        m_ObjectData[id].second.start();

        /*
         * Update object data with position and attitude.
         *
         * Note that we reorder the rotation angles we get from the Vicon system
         * so that they are in the order of yaw, pitch and roll (which seems to
         * be standard).
         */
        using namespace units::length;
        using namespace units::angle;
        m_ObjectData[id].first.update(frameNumber,
                                      std::move(objectName),
                                      millimeter_t(position[0]),
                                      millimeter_t(position[1]),
                                      millimeter_t(position[2]),
                                      radian_t(attitude[2]),
                                      radian_t(attitude[0]),
                                      radian_t(attitude[1]));
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
                                                   0, nullptr, nullptr);

            // If there was an error
            if(bytesReceived == -1) {
                // If this was a timeout, continue
                if(errno == EAGAIN || errno == EINTR) {
                    continue;
                }
                // Otherwise, display error and stop
                else {
                    throw OS::Net::NetworkError("Cannot read datagram");
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
                    BOB_ASSERT(itemDataSize == 72);

                    // Read object name
                    char objectName[23];
                    memcpy(&objectName, &buffer[itemOffset + 3], sizeof(objectName));

                    // Read object position
                    Vector3<double> position;
                    memcpy(&position[0], &buffer[itemOffset + 27], 3 * sizeof(double));

                    // Read object attitude
                    Vector3<double> attitude;
                    memcpy(&attitude[0], &buffer[itemOffset + 51], 3 * sizeof(double));

                    // Update item
                    updateObjectData(objectID, frameNumber, objectName, position, attitude);

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
    void *m_ReadUserData;

    std::mutex m_ObjectDataMutex;
    std::vector<std::pair<ObjectDataType, Stopwatch>> m_ObjectData;
};
} // namespace Vicon
} // BoBRobotics
