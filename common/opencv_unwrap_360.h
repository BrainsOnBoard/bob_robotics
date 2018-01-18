#pragma once

// Standard C includes
#include <cassert>
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// OpenCVUnwrap360
//----------------------------------------------------------------------------
class OpenCVUnwrap360
{
public:
    OpenCVUnwrap360()
    {
    }

    OpenCVUnwrap360(const cv::Size &cameraResolution, const cv::Size &unwrappedResolution,
                    double centreX = 0.5, double centreY = 0.5, double inner = 0.1, double outer = 0.5,
                    double offsetRadians = 0.0, bool flip = false)
    {
        create(cameraResolution, unwrappedResolution,
               centreX, centreY, inner, outer,
               offsetRadians, flip);
    }

    void create(const cv::Size &cameraResolution, const cv::Size &unwrappedResolution,
                double centreX = 0.5, double centreY = 0.5, double inner = 0.1, double outer = 0.5,
                double offsetRadians = 0.0, bool flip = false)
    {
        // convert relative (0.0 to 1.0) to absolute pixel values
        const int centreXPixel = (int)round((double)cameraResolution.width * centreX);
        const int centreYPixel = (int)round((double)cameraResolution.height * centreY);
        const int innerPixel = (int)round((double)cameraResolution.height * inner);
        const int outerPixel = (int)round((double)cameraResolution.height * outer);

        // Create x and y pixel maps
        m_UnwrapMapX.create(unwrappedResolution, CV_32FC1);
        m_UnwrapMapY.create(unwrappedResolution, CV_32FC1);

        // Build unwrap maps
        const float pi = 3.141592653589793238462643383279502884f;
        for (int i = 0; i < unwrappedResolution.height; i++) {
            for (int j = 0; j < unwrappedResolution.width; j++) {
                // Get i as a fraction of unwrapped height, flipping if desires
                const float iFrac = flip ?
                    1.0 - ((float)i / (float)unwrappedResolution.height)
                    : ((float)i / (float)unwrappedResolution.height);

                // Convert i and j to polar
                const float r = iFrac * (outerPixel - innerPixel) + innerPixel;
                const float th = (((float)j / (float)unwrappedResolution.width) * 2.0f * pi) + offsetRadians;

                // Remap onto sphere
                const float x = centreXPixel - r * sin(th);
                const float y = centreYPixel + r * cos(th);
                m_UnwrapMapX.at<float>(i, j) = x;
                m_UnwrapMapY.at<float>(i, j) = y;
            }
        }
    }

    void unwrap(const cv::Mat &input, cv::Mat &output)
    {
        cv::remap(input, output, m_UnwrapMapX, m_UnwrapMapY, cv::INTER_NEAREST);
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;
};