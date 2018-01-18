#pragma once

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

// Common includes
#include "opencv_unwrap_360.h"
#include "v4l_camera.h"

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

    bool capture(cv::Mat &output)
    {
        // Read data and size (in bytes) from camera
        // **NOTE** these pointers are only valid within one frame
        void *data = nullptr;
        uint32_t sizeBytes = 0;
        if(Video4LinuxCamera::capture(data, sizeBytes)) {
            // Check frame size is correct
            assert(sizeBytes == (getWidth() * getHeight() * sizeof(uint16_t)));

            // Add OpenCV header to data
            cv::Mat bayer(getHeight(), getWidth(), CV_16UC1, data);

            // Overwrite the IR data with duplicated green channel to convert to standard RGGB Bayer format
            fillRGGB(bayer);

            // Use OpenCV to demosaic bayer image (the camera's actual Bayer format is BG,
            // but as OpenCV uses BGR rather than RGB, we use RG Bayer format to take this into account
            cv::demosaicing(bayer, m_Demosaiced16, cv::COLOR_BayerRG2BGR);

            // Rescale to 8-bit per channel for output
            cv::convertScaleAbs(m_Demosaiced16, output, 0.249023);

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
