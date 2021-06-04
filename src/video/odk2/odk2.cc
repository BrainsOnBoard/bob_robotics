// BoB robotics includes
#include "video/odk2/odk2.h"

// Third party includes
#include "plog/Log.h"

// Standard C++ includes
#include <stdexcept>

using namespace units::angle;

//------------------------------------------------------------------------
// BoBRobotics::Video::ODK2
//------------------------------------------------------------------------
namespace BoBRobotics {
namespace Video {
ODK2::ODK2(const std::string &hostIP, const std::string &remoteIP)
:   m_Mask(BAND_HEIGHT, BAND_WIDTH, CV_8UC1)
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
    // **NOTE** all buffers are required whether we want the data or not
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
bool ODK2::readFrame(cv::Mat &outFrame)
{
    // Ensure output image is correctly formatted
    // **NOTE** we do this even if there's no image data so output frame is always right size
    outFrame.create(BAND_HEIGHT, BAND_WIDTH, CV_8UC3);

    // Obtain output buffer lock
    std::lock_guard<std::mutex> l(m_OutputBufsMutex);

    // If there's a new frame
    if(m_RawFrameBuf.timestampUs > m_LastRawFrameTimestep) {
        using namespace units::literals;

        // Update last timestep
        m_LastRawFrameTimestep = m_RawFrameBuf.timestampUs;

        // Get reference to quaternion to save typing
        const auto &q = m_RawFrameBuf.quaternion;

        // Calculate heading from quaternion
        const radian_t heading(std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)),
                                          1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))));
        const uint32_t headingIdx = turn_t(heading + 180_deg).value() * BAND_WIDTH;

        LOGD << "Processing frame heading = " << headingIdx;

        // Loop through rows of horizontal band
        for(int i = 0; i < BAND_HEIGHT; i++) {
            // Get pointer to rows in outFrame and mask
            // **NOTE** we want to flip output
            cv::Vec3b *outPixel = outFrame.ptr<cv::Vec3b>(BAND_HEIGHT - i - 1);
            uint8_t *outMask = m_Mask.ptr<uint8_t>(BAND_HEIGHT - i - 1);

            // Loop through columns of horizontal band
            for (int j = 0; j < BAND_WIDTH; j++) {
                const int bandIdx = (i * BAND_WIDTH) + ((j + headingIdx) % BAND_WIDTH);

                // Get fourpi index to map to band pixel
                const int fourpiIdx = band_to_fourpi(bandIdx, bandType_horz);

                // Create pixel into OpenCV image
                const cv::Vec3b pixel(m_RawFrameBuf.image[fourpiIdx * 3],
                                      m_RawFrameBuf.image[(fourpiIdx * 3) + 1],
                                      m_RawFrameBuf.image[(fourpiIdx * 3) + 2]);

                // Write pixel to OpenCV image
                *outPixel++ = pixel;

                // If pixel is black, clear mask otherwise set
                *outMask++ = (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) ? 0 : 255;
            }
        }

        return true;
    }
    else {
        return false;
    }
}
//------------------------------------------------------------------------
std::array<degree_t, 3> ODK2::getEulerAngles() const
{
    // Obtain lock and get copy of fused IMU quaternion
    m_OutputBufsMutex.lock();
    const auto q = m_IMUBuf.q;
    m_OutputBufsMutex.unlock();

    // **NOTE** this whole process should be doable using Eigen but I can't make it work so
    // used https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // Calculate heading from quaternion
    const radian_t heading(std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)),
                                      1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z)))));

    // Calculate roll from quaternion
    const radian_t roll(std::atan2(2.0 * ((q.w * q.x) + (q.y * q.z)),
                                   1.0 - (2.0 * ((q.x * q.x) + (q.y * q.y)))));

    // Calculate pitch from quaternion
    const double sinp = 2.0 * ((q.w * q.y) - (q.z * q.x));
    const radian_t pitch((std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp));

    // To match BN055, we want to return in heading, roll, pitch
    return {heading, roll, pitch};
}
//------------------------------------------------------------------------
void ODK2::readThread()
{
    while(!m_ShouldQuit) {
        // Lock output mutex and read packets from driver
        // **NOTE** because this is a C function it won't throw so no need to use a std::lock_guard
        m_OutputBufsMutex.lock();
        const int ret = devkit_driver_run_rx(&m_State, &m_OutputBufs);
        m_OutputBufsMutex.unlock();

        // No data
        if(ret == 0) {
            LOGD << "No data";
        }
        // Error
        else if (ret < 0) {
            if (ret == DEVKIT_DRIVER_RX_TIMEOUT) {
                LOGW << "RX timeout";
            }
            else {
                throw std::runtime_error("Error in devkit driver RX (" + std::to_string(ret) + ")");
            }
        }
        // Valid data
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
