// BoB robotics includes
#include "video/odk2/odk2.h"

// Third party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Standard C++ includes
#include <stdexcept>

//------------------------------------------------------------------------
// BoBRobotics::Video::ODK2
//------------------------------------------------------------------------
namespace BoBRobotics {
namespace Video {
ODK2::ODK2(const std::string &hostIP, const std::string &remoteIP)
{
    // Build config
    devkitDriverConfig_t config = {};
    strncpy(config.hostIP, hostIP.c_str(), sizeof(config.hostIP) - 1);
    strncpy(config.remoteIP, remoteIP.c_str(), sizeof(config.remoteIP) - 1);
    config.port = 50102;

    // Initialise dev-kit
    const int ret = devkit_driver_init(&m_State, &config);
    if (ret != 0) {
        throw std::runtime_error("Error initialising devkit driver (" + std::to_string(ret) + ")");
    }

    // Configure output buffers data structure
    m_OutputBufs.rawFrame = &m_RawFrameBuf;
    m_OutputBufs.flowFrame = &m_FlowFrameBuf;
    m_OutputBufs.rawCamera = &m_RawCameraBuf;
    m_OutputBufs.imu = &m_IMUBuf;
    m_OutputBufs.state = &m_StateBuf;

    // Clear atomic stop flag and start read thread
    m_ShouldQuit = false;
    m_ReadThread = std::thread(&ODK2::readThread, this);
}
//------------------------------------------------------------------------
ODK2::~ODK2()
{
    // If thread is joinable, set quit flag and join
    if(m_ReadThread.joinable()) {
        m_ShouldQuit = true;
        m_ReadThread.join();
    }
}
//------------------------------------------------------------------------
std::string ODK2::getCameraName() const
{
    return "Opteran DevKit";
}
//------------------------------------------------------------------------
cv::Size ODK2::getOutputSize() const
{
    // For the purposes of BoB robotics we just use the equitorial band of the ODK2
    return cv::Size(BAND_WIDTH, BAND_HEIGHT);
}
//------------------------------------------------------------------------
bool ODK2::readFrame(cv::Mat &outFrame)
{
    std::lock_guard<std::mutex> l(m_OutputBufMutex);

    // If there's a new frame
    if(m_RawFrameBuf.timestampUs > m_LastRawFrameTimestep) {
        using namespace units::angle;
        using namespace units::literals;

        // Ensure output image is correctly formatted
        outFrame.create(BAND_HEIGHT, BAND_WIDTH, CV_8UC3);

        // Update last timestep
        m_LastRawFrameTimestep = m_RawFrameBuf.timestampUs;

        // Extract quaternion components from frame
        const float q0 = m_RawFrameBuf.quaternion.w;
        const float q1 = m_RawFrameBuf.quaternion.x;
        const float q2 = m_RawFrameBuf.quaternion.y;
        const float q3 = m_RawFrameBuf.quaternion.z;

        // Calculate heading from quaternion
        const radian_t heading(atan2(2.0 * ((q0 * q3) + (q1 * q2)), 1.0 - (2.0 * ((q2 * q2) + (q3 * q3)))));
        const uint32_t headingIdx = turn_t(heading + 180_deg).value() * BAND_WIDTH;

        LOGD << "Processing frame heading=" << headingIdx;

        // Loop through pixels in horizontal band
        for(int i = 0; i < BAND_HEIGHT; i++) {
            uint8_t *pixel = outFrame.ptr<uint8_t>(i);

            for (int j = 0; j < BAND_WIDTH; j++) {
                const int bandIdx = (i * BAND_WIDTH) + ((j + headingIdx) % BAND_WIDTH);

                // Get fourpi index to map to band pixel
                const int fourpiIdx = band_to_fourpi(bandIdx, bandType_horz);

                // Copy indexed pixel into OpenCV image
                *pixel++ = m_RawFrameBuf.image[fourpiIdx * 3];
                *pixel++ = m_RawFrameBuf.image[(fourpiIdx * 3) + 1];
                *pixel++ = m_RawFrameBuf.image[(fourpiIdx * 3) + 2];
            }
        }
        return true;
    }
    else {
        return false;
    }
}
//------------------------------------------------------------------------
void ODK2::readThread()
{
    while(!m_ShouldQuit) {
        m_OutputBufMutex.lock();
        const int ret = devkit_driver_run_rx(&m_State, &m_OutputBufs);
        m_OutputBufMutex.unlock();

        // Error
        if(ret == 0) {
            LOGV << "No data";
        }
        else if (ret < 0) {
            if (ret == DEVKIT_DRIVER_RX_TIMEOUT) {
                LOGW << "RX timeout";
            }
            else {
                throw std::runtime_error("Error in devkit driver RX (" + std::to_string(ret) + ")");
            }
        }
        else if (ret > 0) {
            if (ret == DEVKIT_DRIVER_RAW_FRAME_VALID) {
                LOGD << "Got raw frame with timestamp " << m_RawFrameBuf.timestampUs;
            }
            else if (ret == DEVKIT_DRIVER_FLOW_FRAME_VALID) {
                LOGD << "Got flow frame with timestamp " << m_FlowFrameBuf.timestampUs;
            }
            else if (ret == DEVKIT_DRIVER_RAW_CAMERA_VALID) {
                LOGD << "Got raw camera frame with timestamp " << m_RawCameraBuf.timestampUs;
            }
            else if (ret == DEVKIT_DRIVER_IMU_VALID) {
                LOGD << "Got IMU sample with timestamp " << m_IMUBuf.timestampUs;
            }
            else if (ret == DEVKIT_DRIVER_STATE_VALID) {
                LOGD << "Got state estimate with timestamp " << m_StateBuf.timestampUs;
            }
            else {
                LOGW << "RX unknown frame";
            }
        }
    }
}

} // Video
} // BoBRobotics
