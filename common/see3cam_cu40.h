#pragma once

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

// Common includes
#include "opencv_unwrap_360.h"
#include "v4l_camera.h"

/* NEON version of captureSuperPixel - fun but doesn't actually help performance
// Create tables to use for shuffling BGIR data into BGR
//                              b0, g0, r0| b1, g1, r1| b2, g2
const uint8x8_t blockShuffleA = {0, 1,  9,  2,  3,  11, 4,  5};
//                              r2| b3, g3, r3|
const uint8x8_t blockShuffleB = {13,6,  7,  15};

// Loop through input pixels in 8x2 blocks i.e. 4 Bayer pixels
for(unsigned int y = 0; y < inputHeight; y += 2) {
    // Get pointers to start of both rows of Bayer data and output RGB data
    const uint16_t *inBG16Start = &bayerData[y * inputWidth];
    const uint16_t *inIR16Start = &bayerData[(y + 1) * inputWidth];
    uint8_t *outRGBStart = output.ptr(y / 2);
    
    for(unsigned int x = 0; x < inputWidth; x += 8) {
        // Load 8x2 block into two 128-bit registers and advance each pointer to next block
        // b0, g0 | b1, g1 | b2, g2 | b3, g3 |
        uint16x8_t inBG16 = vld1q_u16(inBG16Start += 8);
        // i0, r0 | i1, r1 | i2, r2 | i3, r3 |
        uint16x8_t inIR16 = vld1q_u16(inIR16Start += 8);
        
        // Shift each 10-bit value right by 2
        inBG16 = vshrq_n_u16(inBG16, 2);
        inIR16 = vshrq_n_u16(inIR16, 2);
        
        // Convert to 8 bits and stack into a pair of 64-bit registers
        uint8x8x2_t outBGIR;
        outBGIR.val[0] = vmovn_u16(inBG16);
        outBGIR.val[1] = vmovn_u16(inIR16);
        
        // Use tables to shuffle each 64-bit register worth of Bayer data into RGB
        uint8x8_t outA = vtbl2_u8(outBGIR, blockShuffleA);
        uint8x8_t outB = vtbl2_u8(outBGIR, blockShuffleB);
        
        // Combine the two shuffled vectors together into one 128-bit register full of RGB data
        uint8x16_t out8 = vcombine_u8(outA, outB);
        
        // Write this back to output array and advance pointer
        //if(y != (inputHeight-1) || x != (inputWidth - 1)) {
            vst1q_u8(outRGBStart += (4 * 3), out8);
        //}
    }
}*/

//------------------------------------------------------------------------
// See3CAM_CU40
//------------------------------------------------------------------------
class See3CAM_CU40 : public Video4LinuxCamera
{
public:
    enum class Resolution : uint64_t
    {
        _672x380    = (672ULL | ( 380ULL << 32)),
        _1280x720   = (1280ULL | (720ULL << 32)),
        _1920x1080  = (1920ULL | (1080ULL << 32)),
        _2688x1520  = (2688ULL | (1520ULL << 32)),
    };

    See3CAM_CU40()
    {
    }

    See3CAM_CU40(const std::string &device, Resolution res, bool resetToDefaults = true)
    {
        if(!open(device, res, resetToDefaults)) {
            throw std::runtime_error("Cannot open See3CAM_CU40");
        }
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool open(const std::string &device, Resolution res, bool resetToDefaults = true)
    {
        // Cache resolution
        m_Resolution = res;

        // Create additional frame to hold demosaiced frames
        m_Demosaiced16.create(getHeight(), getWidth(), CV_16UC3);

        // If camera was opened successfully
        if(Video4LinuxCamera::open(device, getWidth(), getHeight(), V4L2_PIX_FMT_Y16))
        {
            // Query camera controls
            if(!queryControl(V4L2_CID_BRIGHTNESS, m_BrightnessControl)
                || !queryControl(V4L2_CID_EXPOSURE_ABSOLUTE, m_ExposureControl))
            {
                return false;
            }

            // If we should reset camera to default settings, do so
            if(resetToDefaults) {
                if(!setControlValue(V4L2_CID_BRIGHTNESS, m_BrightnessControl.default_value)
                    || !setControlValue(V4L2_CID_EXPOSURE_ABSOLUTE, m_ExposureControl.default_value))
                {
                    return false;
                }
            }

            return true;
        }
        else {
            return false;
        }
    }

    bool captureSuperPixel(cv::Mat &output)
    {
        const unsigned int inputWidth = getWidth();
        const unsigned int inputHeight = getHeight();
        assert(output.cols == inputWidth / 2);
        assert(output.rows == inputHeight / 2);
   
        // Read data and size (in bytes) from camera
        // **NOTE** these pointers are only valid within one frame
        void *data = nullptr;
        uint32_t sizeBytes = 0;
        if(Video4LinuxCamera::capture(data, sizeBytes)) {
            // Check frame size is correct
            assert(sizeBytes == (inputWidth * inputHeight * sizeof(uint16_t)));
            const uint16_t *bayerData = reinterpret_cast<uint16_t*>(data);
        
            // Loop through bayer pixels
            for(unsigned int y = 0; y < inputHeight; y += 2) {
                // Get pointers to start of both rows of Bayer data and output RGB data
                const uint16_t *inBG16Start = &bayerData[y * inputWidth];
                const uint16_t *inR16Start = &bayerData[((y + 1) * inputWidth) + 1];
                uint8_t *outRGBStart = output.ptr(y / 2);
                for(unsigned int x = 0; x < inputWidth; x += 2)
                {
                    // Read Bayer pixels
                    const uint16_t b = *(inBG16Start++) >> 2;
                    const uint16_t g = *(inBG16Start++) >> 2;
                    const uint16_t r = *(inR16Start += 2) >> 2;
                    
                    // Write back to RGB
                    *(outRGBStart++) = (uint8_t)b;
                    *(outRGBStart++) = (uint8_t)g;
                    *(outRGBStart++) = (uint8_t)r;
                }
            }

            return true;
        }
        else {
            return false;
        }
    }

    bool setBrightness(int32_t brightness)
    {
        return setControlValue(V4L2_CID_BRIGHTNESS, std::max(m_BrightnessControl.minimum, std::min(m_BrightnessControl.maximum, brightness)));
    }

    bool setExposure(int32_t exposure)
    {
        return setControlValue(V4L2_CID_EXPOSURE_ABSOLUTE, std::max(m_ExposureControl.minimum, std::min(m_ExposureControl.maximum, exposure)));
    }

    unsigned int getWidth() const
    {
        return (static_cast<uint64_t>(m_Resolution) & 0xFFFFFFFF);
    }

    unsigned int getHeight() const
    {
        return ((static_cast<uint64_t>(m_Resolution) >> 32) & 0xFFFFFFFF);
    }

    //------------------------------------------------------------------------
    // Static API
    //------------------------------------------------------------------------
    static OpenCVUnwrap360 createUnwrapper(const cv::Size &camRes, const cv::Size &unwrapRes)
    {
        return OpenCVUnwrap360(camRes, unwrapRes,
                               0.5, 0.434722222, 0.176388889, 0.381944444, 1.570796327, true);
    }
    
private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    // Overwrite the IR data in the raw bayer image with a
    // duplicated green channel to form standard RGGB Bayer format
    void fillRGGB(cv::Mat &bayer)
    {
        for (int row = 0; row < bayer.rows; row+=2)
        {
            for (int col = 0; col < bayer.cols; col+=2)
            {
                bayer.at<uint16_t>(row + 1, col) = bayer.at<uint16_t>(row, col + 1);
            }
        }
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    Resolution m_Resolution;
    v4l2_queryctrl m_BrightnessControl;
    v4l2_queryctrl m_ExposureControl;

    cv::Mat m_Demosaiced16;
};
