#pragma once

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Standard C includes
#include <cassert>
#include <cstring>

// Networking includes
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
#include "../common/pose.h"

using namespace units::math;
using namespace units::time;
using namespace units::velocity;

using namespace BoBRobotics::Pose;

namespace BoBRobotics {
//----------------------------------------------------------------------------
// Vicon Typedefines
//----------------------------------------------------------------------------
namespace Vicon
{
//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and attitude
class ObjectData
{
public:
    ObjectData()
      : m_FrameNumber{ 0 }
      , m_Position{ 0_mm, 0_mm, 0_mm }
      , m_Attitude{ 0_rad, 0_rad, 0_rad }
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        // Cache frame number
        m_FrameNumber = frameNumber;

        // Copy vectors into class
        m_Position[0] = x;
        m_Position[1] = y;
        m_Position[2] = z;
        m_Attitude[0] = yaw;
        m_Attitude[1] = pitch;
        m_Attitude[2] = roll;
    }

    uint32_t getFrameNumber() const
    {
        return m_FrameNumber;
    }

    template <class LengthUnit = millimeter_t>
    Triple<LengthUnit> getPosition()
    {
        return makeUnitTriple<LengthUnit>(m_Position);
    }

    template <class AngleUnit = radian_t>
    Triple<radian_t> getAttitude()
    {
        return makeUnitTriple<AngleUnit>(m_Attitude);
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber;
    Array3<millimeter_t> m_Position;
    Array3<radian_t> m_Attitude;
};

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
public:
    ObjectDataVelocity() : m_Velocity{0_mps, 0_mps, 0_mps}
    {}

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z,
                radian_t yaw, radian_t pitch, radian_t roll)
    {
        const Triple<millimeter_t> position {x, y, z};
        constexpr millisecond_t frameS = 10_ms;
        constexpr millisecond_t smoothingS = 30_ms;

        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const auto deltaS = frameS * deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - exp(-deltaS / smoothingS);

        // Calculate instantaneous velocity
        const auto oldPosition = getPosition<>();
        Array3<meters_per_second_t> instVelocity;
        const auto calcVelocity = [deltaS](auto curr, auto prev) {
            return (curr - prev) / deltaS;
        };
        instVelocity[0] = calcVelocity(getX(position), getX(oldPosition));
        instVelocity[1] = calcVelocity(getY(position), getZ(oldPosition));
        instVelocity[2] = calcVelocity(getZ(position), getZ(oldPosition));

        // Exponentially smooth velocity
        const auto smoothVelocity = [alpha](auto inst, auto prev) {
            return (alpha * inst) + ((1.0 - alpha) * prev);
        };
        std::transform(std::begin(instVelocity), std::end(instVelocity),
                       std::begin(m_Velocity), std::begin(m_Velocity),
                       smoothVelocity);

        // Superclass
        ObjectData::update(frameNumber, x, y, z, yaw, pitch, roll);
    }

    template <class VelocityUnit = meters_per_second_t>
    Triple<VelocityUnit> getVelocity() const
    {
        return makeUnitTriple<VelocityUnit>(m_Velocity);
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Array3<meters_per_second_t> m_Velocity;
};

//----------------------------------------------------------------------------
// Vicon::UDPClient
//----------------------------------------------------------------------------
// Receiver for Vicon UDP streams
template<typename ObjectDataType = ObjectData>
class UDPClient
{
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
            throw std::runtime_error("Invalid object id: " + std::to_string(id));
        }
    }

private:
    //----------------------------------------------------------------------------
    // Private API
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

    void updateObjectData(unsigned int id, uint32_t frameNumber,
                          const Array3<double> &position,
                          const Array3<double> &attitude)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if(id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }

        /*
         * Update object data with position and attitude.
         * 
         * Note that we reorder the rotation angles we get from the Vicon system
         * so that they are in the order of yaw, pitch and roll (which seems to
         * be standard).
         */
        m_ObjectData[id].update(frameNumber,
                                units::make_unit<millimeter_t>(position[0]),
                                units::make_unit<millimeter_t>(position[1]),
                                units::make_unit<millimeter_t>(position[2]),
                                units::make_unit<radian_t>(attitude[2]),
                                units::make_unit<radian_t>(attitude[0]),
                                units::make_unit<radian_t>(attitude[1]));
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

                    // Read object position
                    Array3<double> position;
                    memcpy(&position[0], &buffer[itemOffset + 27], sizeof(position));

                    // Read object attitude
                    Array3<double> attitude;
                    memcpy(&attitude[0], &buffer[itemOffset + 51], sizeof(attitude));

                    // Update item
                    updateObjectData(objectID, frameNumber, position, attitude);

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
    std::vector<ObjectDataType> m_ObjectData;
};
} // namespace Vicon
} // BoBRobotics
