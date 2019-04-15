#ifdef __linux__
// BoB robotics includes
#include "common/logging.h"
#include "video/see3cam_cu40.h"

// Standard C++
#include <chrono>
#include <numeric>
#include <thread>

namespace BoBRobotics {
namespace Video {

See3CAM_CU40::See3CAM_CU40()
{
}

See3CAM_CU40::See3CAM_CU40(const std::string &device, Resolution res, bool resetToDefaults)
{
    open(device, res, resetToDefaults);
}

std::string
See3CAM_CU40::getCameraName() const
{
    return "see3cam";
}

bool
See3CAM_CU40::readFrame(cv::Mat &outFrame)
{
    // Try to read from camera and throw error if it fails
    outFrame.create(getSuperPixelSize(), CV_8UC3);
    captureSuperPixelWBU30(outFrame);
    return true;
}

bool
See3CAM_CU40::readGreyscaleFrame(cv::Mat &outFrame)
{
    // Try to read from camera and throw error if it fails
    outFrame.create(getSuperPixelSize(), CV_8UC1);
    captureSuperPixelGreyscale(outFrame);
    return true;
}

cv::Size
See3CAM_CU40::getOutputSize() const
{
    return getSuperPixelSize();
}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
void
See3CAM_CU40::open(const std::string &device,
                   Resolution res,
                   bool resetToDefaults)
{
    // Cache resolution
    m_Resolution = res;

    // If camera was opened successfully
    Video4LinuxCamera::open(device, getWidth(), getHeight(), V4L2_PIX_FMT_Y16);

    // Query camera controls
    queryControl(V4L2_CID_BRIGHTNESS, m_BrightnessControl);
    queryControl(V4L2_CID_EXPOSURE_ABSOLUTE, m_ExposureControl);

    // If we should reset camera to default settings, do so
    if (resetToDefaults) {
        setControlValue(V4L2_CID_BRIGHTNESS, m_BrightnessControl.default_value);
        setControlValue(V4L2_CID_EXPOSURE_ABSOLUTE, m_ExposureControl.default_value);
    }
}

void
See3CAM_CU40::captureSuperPixel(cv::Mat &output)
{
    captureSuperPixel<PixelScale>(output);
}

void
See3CAM_CU40::captureSuperPixelClamp(cv::Mat &output)
{
    captureSuperPixel<PixelClamp>(output);
}

void
See3CAM_CU40::captureSuperPixelWBCoolWhite(cv::Mat &output)
{
    captureSuperPixel<WhiteBalanceCoolWhite>(output);
}

void
See3CAM_CU40::captureSuperPixelWBU30(cv::Mat &output)
{
    captureSuperPixel<WhiteBalanceU30>(output);
}

void
See3CAM_CU40::captureSuperPixelGreyscale(cv::Mat &output)
{
    // Check that output size is suitable for super-pixel output i.e. a
    // quarter input size
    const unsigned int inputWidth = getWidth();
    const unsigned int inputHeight = getHeight();
    BOB_ASSERT(output.cols == (int) inputWidth / 2);
    BOB_ASSERT(output.rows == (int) inputHeight / 2);
    BOB_ASSERT(output.type() == CV_8UC1);

    // Read data and size (in bytes) from camera
    // **NOTE** these pointers are only valid within one frame
    void *data = nullptr;
    uint32_t sizeBytes = Video4LinuxCamera::capture(data);

    // Check frame size is correct
    BOB_ASSERT(sizeBytes == (inputWidth * inputHeight * sizeof(uint16_t)));
    const uint16_t *bayerData = reinterpret_cast<uint16_t *>(data);

    // Loop through bayer pixels
    for (unsigned int y = 0; y < inputHeight; y += 2) {
        // Get pointers to start of both rows of Bayer data and output
        // RGB data
        const uint16_t *inBG16Start = &bayerData[y * inputWidth];
        const uint16_t *inR16Start =
                &bayerData[((y + 1) * inputWidth) + 1];
        uint8_t *outStart = output.ptr(y / 2);
        for (unsigned int x = 0; x < inputWidth; x += 2) {
            // Read Bayer pixels
            const uint16_t b = *(inBG16Start++);
            const uint16_t g = *(inBG16Start++);
            const uint16_t r = *inR16Start;
            inR16Start += 2;

            // Add channels together and divide by 3 to take average and
            // 4 to rescale from 10-bit per-channel to 8-bit
            const uint32_t gray = (b + g + r) / (3 * 4);

            // Write back to BGR
            *(outStart++) = (uint8_t) gray;
        }
    }
}

// Calculates entropy, either from whole frame or within subset specified by
// mask
// **NOTE** this uses full 10-bit sensor range for calculation
float
See3CAM_CU40::calculateImageEntropy(const cv::Mat &mask)
{
    const unsigned int inputWidth = getWidth();
    const unsigned int inputHeight = getHeight();

    // Check validity of mask
    const bool noMask = (mask.cols == 0 && mask.rows == 0);
    BOB_ASSERT(noMask || (mask.cols == (int) (inputWidth / 2) &&
                          mask.rows == (int) (inputHeight / 2)));
    BOB_ASSERT(noMask || mask.type() == CV_8UC1);

    // Read data and size (in bytes) from camera
    // **NOTE** these pointers are only valid within one frame
    void *data = nullptr;
    uint32_t sizeBytes = Video4LinuxCamera::capture(data);

    // Check frame size is correct
    BOB_ASSERT(sizeBytes == (inputWidth * inputHeight * sizeof(uint16_t)));
    const uint16_t *bayerData = reinterpret_cast<uint16_t *>(data);

    // Zero a 10-bit RGB histogram for each colour channel
    unsigned int hist[3][1024];
    std::fill_n(hist[0], 1024, 0);
    std::fill_n(hist[1], 1024, 0);
    std::fill_n(hist[2], 1024, 0);

    // Loop through bayer pixels
    unsigned int numPixels = 0;
    for (unsigned int y = 0; y < inputHeight; y += 2) {
        // Get pointers to start of both rows of Bayer data and output
        // RGB data
        const uint16_t *inBG16Start = &bayerData[y * inputWidth];
        const uint16_t *inR16Start =
                &bayerData[((y + 1) * inputWidth) + 1];
        for (unsigned int x = 0; x < inputWidth; x += 2) {
            // Read Bayer pixels
            const uint16_t b = *(inBG16Start++);
            const uint16_t g = *(inBG16Start++);
            const uint16_t r = *inR16Start;
            inR16Start += 2;

            // If no mask is in use or this pixel isn't masked
            // **NOTE** divide by two as x and y are in terms of Bayer
            // pixels
            if (noMask || mask.at<uint8_t>(y / 2, x / 2) > 0) {
                // Increment histogram bins
                hist[0][r]++;
                hist[1][g]++;
                hist[2][b]++;

                // Increment totals
                numPixels++;
            }
        }
    }

    // Check pixel count
    BOB_ASSERT(!noMask || numPixels == (inputWidth * inputHeight / 4));

    // Sum together entropy for each colour channel
    float entropy = 0.0f;
    for (unsigned int c = 0; c < 3; c++) {
        entropy -= std::accumulate(
                std::begin(hist[c]),
                std::end(hist[c]),
                0.0f,
                [numPixels](float acc, unsigned int h) {
                    if (h == 0) {
                        return acc;
                    } else {
                        const float p = (float) h / (float) numPixels;
                        return acc + (p * std::log(p));
                    }
                });
    }
    return entropy;
}

// Automatically configure camera exposure and brightness to optimise image
// quality Use of mask inspired by "Nourani-Vatani & Roberts (2007).
// Automatic Camera Exposure Control." Uses image-entropy based algorithm
// described by "Lu, Zhang et al. (2010). Camera parameters auto-adjusting
// technique for robust robot vision."
void
See3CAM_CU40::autoExposure(const cv::Mat &mask, int brightnessExposureConstant)
{
    // Configure initial brightness and exposure values
    int32_t brightness = m_BrightnessControl.minimum;
    int32_t exposure = std::max(m_ExposureControl.minimum,
                                brightness * brightnessExposureConstant);
    BOB_ASSERT(exposure < m_ExposureControl.maximum);

    // Loop until we've
    int32_t previousBrightness = 0;
    int32_t previousExposure = 0;
    float previousEntropy = 0.0f;
    while (brightness < m_BrightnessControl.maximum &&
           exposure < m_ExposureControl.maximum) {
        // Set exposure and brightness
        setExposure(exposure);
        setBrightness(brightness);

        // Wait for change to take effect
        // **NOTE** delay  found experimentally
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Throw away frame so new frame is captured AFTER setting change
        // **NOTE** this is required because of double-buffering in
        // Video4LinuxCamera
        void *data = nullptr;
        capture(data);

        // Calculate image entropy
        const float entropy = calculateImageEntropy(mask);

        // If we have passed entropy peak
        if (entropy < previousEntropy) {
            LOG_INFO << "Optimal exposure and brightness settings found: "
                        "exposure="
                     << previousExposure
                     << ", brightness=" << previousBrightness;
            setExposure(previousExposure);
            setBrightness(previousBrightness);
            return;
        }
        // Otherwise
        else {
            // Update previous state
            previousEntropy = entropy;
            previousBrightness = brightness;
            previousExposure = exposure;

            // Follow path
            // **NOTE** step size found experimentally
            brightness += (entropy < 15.0f) ? 8 : 2;
            exposure = (brightness * brightnessExposureConstant);
        }
    }
}

void
See3CAM_CU40::setBrightness(int32_t brightness)
{
    setControlValue(V4L2_CID_BRIGHTNESS,
                    std::max(m_BrightnessControl.minimum,
                             std::min(m_BrightnessControl.maximum, brightness)));
}

void
See3CAM_CU40::setExposure(int32_t exposure)
{
    setControlValue(V4L2_CID_EXPOSURE_ABSOLUTE,
                    std::max(m_ExposureControl.minimum,
                             std::min(m_ExposureControl.maximum, exposure)));
}

int32_t
See3CAM_CU40::getBrightness() const
{
    return getControlValue(V4L2_CID_BRIGHTNESS);
}

int32_t
See3CAM_CU40::getExposure() const
{
    return getControlValue(V4L2_CID_EXPOSURE_ABSOLUTE);
}

unsigned int
See3CAM_CU40::getWidth() const
{
    return (static_cast<uint64_t>(m_Resolution) & 0xFFFFFFFF);
}

unsigned int
See3CAM_CU40::getHeight() const
{
    return ((static_cast<uint64_t>(m_Resolution) >> 32) & 0xFFFFFFFF);
}

unsigned int
See3CAM_CU40::getSuperPixelWidth() const
{
    return getWidth() / 2;
}

unsigned int
See3CAM_CU40::getSuperPixelHeight() const
{
    return getHeight() / 2;
}

cv::Size
See3CAM_CU40::getSuperPixelSize() const
{
    return cv::Size(getSuperPixelWidth(), getSuperPixelHeight());
}

bool
See3CAM_CU40::needsUnwrapping() const
{
    return true;
}

cv::Mat
See3CAM_CU40::createBubblescopeMask(const cv::Size &camRes)
{
    cv::Mat mask(camRes, CV_8UC1, cv::Scalar(0, 0, 0));

    // Calculate mask dimensions in pixels
    // **YUCK** as these constants are duplicated from createUnwrapper they
    // should be stuck elsewhere
    const int centreXPixel = (int) round((double) camRes.width * 0.5);
    const int centreYPixel = (int) round((double) camRes.height * 0.461111);
    const int innerPixel = (int) round((double) camRes.height * 0.183333);
    const int outerPixel = (int) round((double) camRes.height * 0.4);

    // Draw outer white circle of mask
    cv::circle(mask,
               cv::Point(centreXPixel, centreYPixel),
               outerPixel,
               cv::Scalar::all(255),
               cv::FILLED);

    // Draw inner black circle
    cv::circle(mask,
               cv::Point(centreXPixel, centreYPixel),
               innerPixel,
               cv::Scalar::all(0),
               cv::FILLED);

    return mask;
}

uint8_t
See3CAM_CU40::PixelScale::getR(uint16_t r, uint16_t, uint16_t)
{
    return getScaled(r);
}
uint8_t
See3CAM_CU40::PixelScale::getG(uint16_t, uint16_t g, uint16_t)
{
    return getScaled(g);
}
uint8_t
See3CAM_CU40::PixelScale::getB(uint16_t, uint16_t, uint16_t b)
{
    return getScaled(b);
}

uint8_t
See3CAM_CU40::PixelScale::getScaled(uint16_t v)
{
    return (uint8_t)(v >> 2);
}

uint8_t
See3CAM_CU40::PixelClamp::getR(uint16_t r, uint16_t, uint16_t)
{
    return getClamped(r);
}
uint8_t
See3CAM_CU40::PixelClamp::getG(uint16_t, uint16_t g, uint16_t)
{
    return getClamped(g);
}
uint8_t
See3CAM_CU40::PixelClamp::getB(uint16_t, uint16_t, uint16_t b)
{
    return getClamped(b);
}
uint8_t
See3CAM_CU40::PixelClamp::getClamped(uint16_t v)
{
    return (uint8_t) std::min<uint16_t>(255, v);
}

uint8_t
See3CAM_CU40::WhiteBalanceCoolWhite::getR(uint16_t r, uint16_t, uint16_t)
{
    // 0.96 (15729)
    return (uint8_t)(((uint32_t) r * 15729) >> 16);
}

uint8_t
See3CAM_CU40::WhiteBalanceCoolWhite::getG(uint16_t, uint16_t g, uint16_t)
{
    return (uint8_t)(g >> 2);
}

uint8_t
See3CAM_CU40::WhiteBalanceCoolWhite::getB(uint16_t, uint16_t, uint16_t b)
{
    // 1.74 (28508)
    return (uint8_t) std::min<uint32_t>(((uint32_t) b * 28508) >> 16,
                                        255);
}

uint8_t
See3CAM_CU40::WhiteBalanceU30::getR(uint16_t r, uint16_t, uint16_t)
{
    // 0.92 (15073)
    return (uint8_t)(((uint32_t) r * 15073) >> 16);
}

uint8_t
See3CAM_CU40::WhiteBalanceU30::getG(uint16_t, uint16_t g, uint16_t)
{
    return (uint8_t)(g >> 2);
}

uint8_t
See3CAM_CU40::WhiteBalanceU30::getB(uint16_t, uint16_t, uint16_t b)
{
    // 1.53 (25068)
    return (uint8_t) std::min<uint32_t>(((uint32_t) b * 25068) >> 16,
                                        255);
}

} // Video
} // BoBRobotics
#endif // linux
