#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "input.h"
#include "v4l_camera.h"

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Video {
// Linux device name
constexpr const char *See3CamDeviceName = "See3CAM_CU40";

//------------------------------------------------------------------------
// BoBRobotics::Video::See3CAM_CU40
//------------------------------------------------------------------------
//! Read video from a See3CAM_CU40
class See3CAM_CU40
  : public Video4LinuxCamera
  , public Input
{
public:
    enum class Resolution : uint64_t
    {
        _672x380 = (672ULL | (380ULL << 32)),
        _1280x720 = (1280ULL | (720ULL << 32)),
        _1920x1080 = (1920ULL | (1080ULL << 32)),
        _2688x1520 = (2688ULL | (1520ULL << 32)),
    };

    See3CAM_CU40();
    See3CAM_CU40(const std::string &device,
                 Resolution res,
                 bool resetToDefaults = true);

    //------------------------------------------------------------------------
    // Video::Input virtuals
    //------------------------------------------------------------------------
    virtual std::string getCameraName() const override;
    virtual bool readFrame(cv::Mat &outFrame) override;
    virtual bool readGreyscaleFrame(cv::Mat &outFrame) override;
    virtual cv::Size getOutputSize() const override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void open(const std::string &device,
              Resolution res,
              bool resetToDefaults = true);
    void captureSuperPixel(cv::Mat &output);
    void captureSuperPixelClamp(cv::Mat &output);
    void captureSuperPixelWBCoolWhite(cv::Mat &output);
    void captureSuperPixelWBU30(cv::Mat &output);
    void captureSuperPixelGreyscale(cv::Mat &output);

    // Calculates entropy, either from whole frame or within subset specified by
    // mask
    // **NOTE** this uses full 10-bit sensor range for calculation
    float calculateImageEntropy(const cv::Mat &mask);

    // Automatically configure camera exposure and brightness to optimise image
    // quality Use of mask inspired by "Nourani-Vatani & Roberts (2007).
    // Automatic Camera Exposure Control." Uses image-entropy based algorithm
    // described by "Lu, Zhang et al. (2010). Camera parameters auto-adjusting
    // technique for robust robot vision."
    void autoExposure(const cv::Mat &mask, int brightnessExposureConstant = 5);

    void setBrightness(int32_t brightness);
    void setExposure(int32_t exposure);
    int32_t getBrightness() const;
    int32_t getExposure() const;
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    unsigned int getSuperPixelWidth() const;
    unsigned int getSuperPixelHeight() const;
    cv::Size getSuperPixelSize() const;
    bool needsUnwrapping() const override;

    //------------------------------------------------------------------------
    // Static API
    //------------------------------------------------------------------------
    static cv::Mat createBubblescopeMask(const cv::Size &camRes);

private:
    //------------------------------------------------------------------------
    // PixelScale
    //------------------------------------------------------------------------
    // Converts 10-bit intensity values to 8-bit by dividing by 4
    class PixelScale
    {
    public:
        static uint8_t getR(uint16_t r, uint16_t, uint16_t);
        static uint8_t getG(uint16_t, uint16_t g, uint16_t);
        static uint8_t getB(uint16_t, uint16_t, uint16_t b);

    private:
        static uint8_t getScaled(uint16_t v);
    };

    //------------------------------------------------------------------------
    // PixelClamp
    //------------------------------------------------------------------------
    // Converts 10-bit intensity values to 8-bit by clamping at 255
    // **NOTE** this is dubious but a)Is what the qtcam example does and b)Can
    // LOOK nicer than PixelScale
    class PixelClamp
    {
    public:
        static uint8_t getR(uint16_t r, uint16_t, uint16_t);
        static uint8_t getG(uint16_t, uint16_t g, uint16_t);
        static uint8_t getB(uint16_t, uint16_t, uint16_t b);

    private:
        static uint8_t getClamped(uint16_t v);
    };

    //------------------------------------------------------------------------
    // WhiteBalanceCoolWhite
    //------------------------------------------------------------------------
    class WhiteBalanceCoolWhite
    {
    public:
        static uint8_t getR(uint16_t r, uint16_t, uint16_t);
        static uint8_t getG(uint16_t, uint16_t g, uint16_t);
        static uint8_t getB(uint16_t, uint16_t, uint16_t b);
    };

    //------------------------------------------------------------------------
    // WhiteBalanceU30
    //------------------------------------------------------------------------
    class WhiteBalanceU30
    {
    public:
        static uint8_t getR(uint16_t r, uint16_t, uint16_t);
        static uint8_t getG(uint16_t, uint16_t g, uint16_t);
        static uint8_t getB(uint16_t, uint16_t, uint16_t b);
    };

    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<typename T>
    void captureSuperPixel(cv::Mat &output)
    {
        // Check that output size is suitable for super-pixel output i.e. a
        // quarter input size
        const unsigned int inputWidth = getWidth();
        const unsigned int inputHeight = getHeight();
        BOB_ASSERT(output.cols == (int) inputWidth / 2);
        BOB_ASSERT(output.rows == (int) inputHeight / 2);
        BOB_ASSERT(output.type() == CV_8UC3);

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
            uint8_t *outRGBStart = output.ptr(y / 2);
            for (unsigned int x = 0; x < inputWidth; x += 2) {
                // Read Bayer pixels
                const uint16_t b = *(inBG16Start++);
                const uint16_t g = *(inBG16Start++);
                const uint16_t r = *inR16Start;
                inR16Start += 2;

                // Write back to BGR
                *(outRGBStart++) = T::getB(r, g, b);
                *(outRGBStart++) = T::getG(r, g, b);
                *(outRGBStart++) = T::getR(r, g, b);
            }
        }
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    Resolution m_Resolution;
    v4l2_queryctrl m_BrightnessControl;
    v4l2_queryctrl m_ExposureControl;
}; // See3Cam_CU40
} // Video
} // BoBRobotics
