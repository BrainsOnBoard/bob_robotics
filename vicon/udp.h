#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/circstat.h"
#include "../common/pose.h"
#include "../common/stopwatch.h"
#include "../common/thread.h"
#include "../os/net.h"

// Standard C++ includes
#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace BoBRobotics
{
namespace Vicon
{
using namespace std::literals;
using namespace units::literals;

//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and attitude
class ObjectData
{
    using radian_t = units::angle::radian_t;
    using millimeter_t = units::length::millimeter_t;

public:
    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(const char(&name)[24],
                uint32_t frameNumber,
                Pose3<millimeter_t, radian_t> pose)
    {
        // Copy name
        // **NOTE** this must already be NULL-terminated
        memcpy(&m_Name[0], &name[0], 24);

        // Log the time when this packet was received
        m_ReceivedTimer.start();

        // Cache frame number
        m_FrameNumber = frameNumber;

        // Copy vectors into class
        m_Pose = pose;
    }

    uint32_t getFrameNumber() const
    {
        return m_FrameNumber;
    }

    template<typename LengthUnit = millimeter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return m_Pose.position();
    }

    template<typename AngleUnit = radian_t>
    std::array<AngleUnit, 3> getAttitude() const
    {
        return convertUnitArray<AngleUnit>(m_Pose.attitude());
    }

    template<typename LengthUnit = millimeter_t, typename AngleUnit = radian_t>
    Pose3<LengthUnit, AngleUnit> getPose() const
    {
        return m_Pose;
    }

    const char *getName() const{ return m_Name; }

    auto timeSinceReceived() const { return m_ReceivedTimer.elapsed(); }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber = 0;
    char m_Name[24];
    Pose3<millimeter_t, radian_t> m_Pose;
    Stopwatch m_ReceivedTimer;
};

// Forward declaration
template<typename ObjectDataType>
class UDPClient;

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
    using radian_t = units::angle::radian_t;
    using radians_per_second_t = units::angular_velocity::radians_per_second_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using millimeter_t = units::length::millimeter_t;
    using second_t = units::time::millisecond_t;

public:
    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(const char(&name)[24],
                uint32_t frameNumber,
                Pose3<millimeter_t, radian_t> pose)
    {
        constexpr second_t frameS = 10_ms;
        constexpr second_t smoothingS = 30_ms;

        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const auto deltaS = frameS * deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - units::math::exp(-deltaS / smoothingS);

        // Calculate velocities (m/s)
        const auto oldPosition = getPosition<>();
        calculateVelocities(pose.position(), oldPosition, m_Velocity, deltaS, alpha, std::minus<millimeter_t>());

        // Calculate angular velocities (rad/s)
        const auto oldAttitude = getAttitude<>();
        const auto circDist = [](auto angle1, auto angle2) {
            return circularDistance(angle1, angle2);
        };
        calculateVelocities(pose.attitude(), oldAttitude, m_AngularVelocity, deltaS, alpha, circDist);

        // Superclass
        // **NOTE** this is at the bottom so OLD position can be accessed
        ObjectData::update(name, frameNumber, pose);
    }

    template<typename VelocityUnit = meters_per_second_t>
    std::array<VelocityUnit, 3> getVelocity() const
    {
        return convertUnitArray<VelocityUnit>(m_Velocity);
    }

    template<typename AngularVelocityUnit = radians_per_second_t>
    std::array<AngularVelocityUnit, 3> getAngularVelocity() const
    {
        return convertUnitArray<AngularVelocityUnit>(m_AngularVelocity);
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::array<meters_per_second_t, 3> m_Velocity{};
    std::array<radians_per_second_t, 3> m_AngularVelocity{};

    template<typename VectorType, typename VelocityType, typename DiffFunc>
    static void calculateVelocities(const VectorType &current,
                                    const VectorType &old,
                                    std::array<VelocityType, 3> &velocityOut,
                                    const second_t &deltaS,
                                    const double alpha,
                                    DiffFunc diff
                                    )
    {
        // Temporary array
        std::array<VelocityType, 3> instVelocity;

        // Calculate instantaneous velocity
        const auto calcVelocity = [deltaS, diff](auto curr, auto prev) {
            return diff(curr, prev) / deltaS;
        };
        std::transform(std::cbegin(current), std::cend(current),
                       std::cbegin(old), std::begin(instVelocity),
                       calcVelocity);

        // Exponentially smooth velocity
        const auto smoothVelocity = [alpha](auto inst, auto prev) {
            return (alpha * inst) + ((1.0 - alpha) * prev);
        };
        std::transform(std::cbegin(instVelocity), std::cend(instVelocity),
                       std::cbegin(velocityOut), std::begin(velocityOut),
                       smoothVelocity);
    }
};

class TimedOutError
  : public std::runtime_error
{
public:
    TimedOutError()
      : std::runtime_error("Timed out waiting for Vicon data")
    {}
};

//----------------------------------------------------------------------------
// BoBRobotics::Vicon::ObjectReference
//----------------------------------------------------------------------------
/**!
 *  \brief Holds a reference to a Vicon object
 *
 * The pose data is updated every time it is read, in contrast to ObjectData,
 * which only contains static data.
 */
template<typename ObjectDataType = ObjectData>
class ObjectReference
{
    using millimeter_t = units::length::millimeter_t;
    using radian_t = units::angle::radian_t;

public:
    ObjectReference(UDPClient<ObjectDataType> &client,
                    const unsigned id,
                    const Stopwatch::Duration timeoutDuration = 10s)
        : m_Client(client)
        , m_Id(id)
        , m_TimeoutDuration(timeoutDuration)
    {}

    template<typename LengthUnit = millimeter_t>
    Vector3<LengthUnit> getPosition() const
    {
        return getData().template getPosition<LengthUnit>();
    }

    template<typename AngleUnit = radian_t>
    std::array<AngleUnit, 3> getAttitude() const
    {
        return getData().template getAttitude<AngleUnit>();
    }

    template<typename LengthUnit = millimeter_t, typename AngleUnit = radian_t>
    auto getPose() const
    {
        const auto data = getData();
        return Pose3<LengthUnit, AngleUnit>(data.template getPosition<LengthUnit>(),
                                            data.template getAttitude<AngleUnit>());
    }

    auto timeSinceReceived() const
    {
        return getData().timeSinceReceived();
    }

    ObjectDataType getData() const
    {
        const auto objectData = m_Client.getObjectData(m_Id);
        if (objectData.timeSinceReceived() > m_TimeoutDuration) {
            throw TimedOutError();
        }
        return objectData;
    }

private:
    UDPClient<ObjectDataType> &m_Client;
    const unsigned m_Id;
    const Stopwatch::Duration m_TimeoutDuration;
};

//----------------------------------------------------------------------------
// BoBRobotics::Vicon::UDPClient
//----------------------------------------------------------------------------
//! Receiver for Vicon UDP streams
template<typename ObjectDataType = ObjectData>
class UDPClient
{
public:


    UDPClient(){}
    UDPClient(uint16_t port)
    {
        connect(port);
    }

    virtual ~UDPClient()
    {
        // Set quit flag
        m_ShouldQuit = true;
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
        m_ReadThread = Thread<>(&UDPClient::readThread, this, socket);
    }

    unsigned int getNumObjects()
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        return m_ObjectData.size();
    }

    unsigned int findObjectID(const std::string &name)
    {
        // Search for object with name
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        auto objIter = std::find_if(m_ObjectData.cbegin(), m_ObjectData.cend(),
            [&name](const ObjectDataType &object)
            {
                return (strcmp(object.getName(), name.c_str()) == 0);
            });

        // If object wasn't found, raise error
        if(objIter == m_ObjectData.cend()) {
            throw std::out_of_range("Cannot find object '" + name + "'");
        }
        // Otherwise, return its index i.e. object ID
        else {
            return (unsigned int)std::distance(m_ObjectData.cbegin(), objIter);
        }
    }

    //! Get current pose information for specified object
    ObjectDataType getObjectData(unsigned int id)
    {
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);
        return m_ObjectData.at(id);
    }

    //! Returns an object whose pose is updated by the Vicon system over time
    auto getObjectReference(unsigned int id,
                            Stopwatch::Duration timeoutDuration = Stopwatch::Duration::max())
    {
        return ObjectReference<ObjectDataType>(*this, id, timeoutDuration);
    }

private:
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void updateObjectData(unsigned int id, const char(&name)[24],
                          uint32_t frameNumber,
                          const std::array<double, 3> &position,
                          const std::array<double, 3> &attitude)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if (id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }

        /*
         * Update object data with position and attitude.
         *
         * Note that we reorder the rotation angles we get from the Vicon system
         * so that they are in the order of yaw, pitch and roll (which seems to
         * be standard).
         */
        using namespace units::length;
        using namespace units::angle;
        m_ObjectData[id].update(name, frameNumber,
                                { { millimeter_t(position[0]),
                                    millimeter_t(position[1]),
                                    millimeter_t(position[2])},
                                  { radian_t(attitude[2]),
                                    radian_t(attitude[0]),
                                    radian_t(attitude[1]) } });
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

                    // Read object name and check it is NULL-terminated
                    char objectName[24];
                    memcpy(&objectName[0], &buffer[itemOffset + 3], 24);
                    BOB_ASSERT(objectName[23] == '\0');

                    // Read object position
                    std::array<double, 3> position;
                    memcpy(&position[0], &buffer[itemOffset + 27], 3 * sizeof(double));

                    // Read object attitude
                    std::array<double, 3> attitude;
                    memcpy(&attitude[0], &buffer[itemOffset + 51], 3 * sizeof(double));

                    // Update item
                    updateObjectData(objectID, objectName, frameNumber, position, attitude);

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
    Thread<> m_ReadThread;
    void *m_ReadUserData;

    std::mutex m_ObjectDataMutex;
    std::vector<ObjectDataType> m_ObjectData;
};
} // namespace Vicon
} // BoBRobotics
