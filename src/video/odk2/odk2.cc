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
:   m_RawFrameUnwrapped(BAND_HEIGHT, BAND_WIDTH, CV_8UC3, 0)
{
    // Build config
    devkitDriverConfig_t config = {};
    strncpy(config.hostIP, hostIP.c_str(), sizeof(config.hostIP) - 1);
    strncpy(config.remoteIP, remoteIP.c_str(), sizeof(config.remoteIP) - 1);
    config.port = 50102;

    // Initialise dev-kit
    const int ret = devkit_driver_init(&m_State, &config);
    if (ret != 0) {
        throw std::runtime_error("Error initrialising devkit driver (" + std::to_string(ret) + ")");
    }

    // Configure output buffers data structure
    m_OutputBufs.rawFrame = &m_RawFrameBuf;
    //m_OutputBufs.flowFrame = &m_FlowFrameBuf;
    //m_OutputBufs.imu = &m_IMUBuf;
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
    using namespace units::angle;
    using namespace units::literals;

    // Always point outFrame at unwrapped raw frame
    outFrame = m_RawFrameUnwrapped;

    // Read
    const int ret = devkit_driver_run_rx(&m_State, &m_OutputBufs);

    // Error
    if (ret < 0) {
        if (ret == DEVKIT_DRIVER_RX_TIMEOUT) {
            LOGW << "RX timeout";
        }
        else {
            throw std::runtime_error("Error in devkit driver RX (" + std::to_string(ret) + ")");
        }
    }
    else if (ret > 0) {
        if (ret == DEVKIT_DRIVER_RAW_FRAME_VALID) {
            // Extract quaternion components from frame
            const float q0 = m_RawFrameBuf.quaternion.w;
            const float q1 = m_RawFrameBuf.quaternion.x;
            const float q2 = m_RawFrameBuf.quaternion.y;
            const float q3 = m_RawFrameBuf.quaternion.z;

            // Calculate heading from quaternion
            const radian_t heading(atan2(2.0 * ((q0 * q3) + (q1 * q2)), 1.0 - (2.0 * ((q2 * q2) + (q3 * q3)))));
            const uint32_t headingIdx = turn_t(heading + 180_deg).value() * BAND_WIDTH;

            for(int i = BAND_HEIGHT - 1; i >= 0; i--) {
                cv::Vec3b *pixel = m_RawFrameUnwrapped.ptr<cv::Vec3b>(i);

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
            //rawCount++;
            //printf("Got raw frame with timestamp %lu! Count = %d, fps = %.2f\r\n", outputBufs.rawFrame->timestampUs, rawCount, rawCount/dt);
        }
        /*else if (ret == DEVKIT_DRIVER_FLOW_FRAME_VALID) {
            flowCount++;
            printf("Got flow frame with timestamp %lu! Count = %d, fps = %.2f\r\n", outputBufs.flowFrame->timestampUs, flowCount, flowCount/dt);
        }
        else if (ret == DEVKIT_DRIVER_IMU_VALID) {
            imuCount++;
            printf("Got IMU sample with timestamp %lu! Count = %d, samples/s = %.2f\r\n", outputBufs.imu->timestampUs, imuCount, imuCount/dt);
            assert(outputBufs.imu->timestampUs < 1000000000);
        }
        else if (ret == DEVKIT_DRIVER_STATE_VALID) {
            stateCount++;
            printf("Got state estimate with timestamp %lu! Count = %d, fps = %.2f\r\n", outputBufs.state->timestampUs, stateCount, stateCount/dt);
        }*/
        else {
            LOGW << "RX unknown frame";
        }
    }

    return false;
}

} // Video
} // BoBRobotics
