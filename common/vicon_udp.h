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

//----------------------------------------------------------------------------
// Vicon Typedefines
//----------------------------------------------------------------------------
namespace Vicon
{
typedef double Vector[3];

//----------------------------------------------------------------------------
// Vicon::ObjectData
//----------------------------------------------------------------------------
//! Simplest object data class - just tracks position and translation
class ObjectData
{
public:
    ObjectData() : m_FrameNumber{0}, m_Translation{0.0, 0.0, 0.0}, m_Rotation{0.0, 0.0, 0.0}
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, const Vector &translation, const Vector &rotation)
    {
        // Cache frame number
        m_FrameNumber = frameNumber;

        // Copy vectors into class
        std::copy(std::begin(translation), std::end(translation), std::begin(m_Translation));
        std::copy(std::begin(rotation), std::end(rotation), std::begin(m_Rotation));
    }

    uint32_t getFrameNumber() const { return m_FrameNumber; }
    const Vector &getTranslation() const{ return m_Translation; }
    const Vector &getRotation() const{ return m_Rotation; }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    uint32_t m_FrameNumber;
    Vector m_Translation;
    Vector m_Rotation;
};

//----------------------------------------------------------------------------
// Vicon::ObjectDataVelocity
//----------------------------------------------------------------------------
//! Object data class which also calculate (un-filtered) velocity
class ObjectDataVelocity : public ObjectData
{
public:
    ObjectDataVelocity() : m_Velocity{0.0, 0.0, 0.0}
    {
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void update(uint32_t frameNumber, const Vector &translation, const Vector &rotation)
    {
        // Calculate time since last frame
        const uint32_t deltaFrames = frameNumber - getFrameNumber();
        const double deltaMs = s_FrameMs * (double)deltaFrames;

        // Calculate exponential smoothing factor
        const double alpha = 1.0 - std::exp(-deltaMs / s_SmoothingMs);

        // Calculate instantaneous velocity
        const Vector &oldTranslation = getTranslation();
        Vector instVelocity;
        std::transform(std::begin(translation), std::end(translation), std::begin(oldTranslation), std::begin(instVelocity),
                       [deltaMs](double curr, double prev){ return (curr - prev) / deltaMs; });

        // Exponentially smooth velocity
        std::transform(std::begin(instVelocity), std::end(instVelocity), std::begin(m_Velocity), std::begin(m_Velocity),
                       [alpha](double inst, double prev){ return (alpha * inst) + ((1.0 - alpha) * prev); });

        // Superclass
        ObjectData::update(frameNumber, translation, rotation);
    }

    const Vector &getVelocity() const{ return m_Velocity; }

    //----------------------------------------------------------------------------
    // Static API
    //----------------------------------------------------------------------------
    static void setSampleRateHz(double sampleRateHz){ s_FrameMs = 1000.0 / sampleRateHz; };
    static void setSmoothingMs(double smoothingMs){ s_SmoothingMs = smoothingMs; }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    Vector m_Velocity;

    //----------------------------------------------------------------------------
    // Static members
    //----------------------------------------------------------------------------
    static double s_FrameMs;
    static double s_SmoothingMs;
};

//----------------------------------------------------------------------------
// Vicon::UDPClient
//----------------------------------------------------------------------------
// Receiver for Vicon UDP streams
template<typename ObjectDataType>
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

    ~UDPClient()
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
        timeval timeout = {
            .tv_sec = 1,
            .tv_usec = 0 };
#endif
        if(setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) {
            std::cerr << "Cannot set socket timeout: " << strerror(errno) << std::endl;
            return false;
        }

        // Create socket address structure
        sockaddr_in localAddress = {
            .sin_family = AF_INET,
            .sin_port = htons(port),
            .sin_addr = { .s_addr = htonl(INADDR_ANY) }};

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

private:
    //----------------------------------------------------------------------------
    // Private API
    //----------------------------------------------------------------------------
    void updateObjectData(unsigned int id, uint32_t frameNumber, const Vector &translation, const Vector &rotation)
    {
        // Lock mutex
        std::lock_guard<std::mutex> guard(m_ObjectDataMutex);

        // If no object data structure has been created for this ID, add one
        if(id >= m_ObjectData.size()) {
            m_ObjectData.resize(id + 1);
        }

        // Update object data with translation and rotation
        m_ObjectData[id].update(frameNumber, translation, rotation);
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
                if(errno == EAGAIN) {
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

                    // Read object translation
                    double translation[3];
                    memcpy(&translation[0], &buffer[itemOffset + 27], 3 * sizeof(double));

                    // Read object rotation
                    double rotation[3];
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

    std::mutex m_ObjectDataMutex;
    std::vector<ObjectDataType> m_ObjectData;
};
} // namespace Vicon
