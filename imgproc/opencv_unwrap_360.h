#pragma once

// Standard C includes
#include <cassert>
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

namespace GeNNRobotics {
namespace ImgProc {
//----------------------------------------------------------------------------
// OpenCVUnwrap360
//----------------------------------------------------------------------------
class OpenCVUnwrap360
{
public:
    OpenCVUnwrap360()
    {}

    OpenCVUnwrap360(const cv::Size &cameraResolution,
                    const cv::Size &unwrappedResolution,
                    double centreX = 0.5,
                    double centreY = 0.5,
                    double inner = 0.1,
                    double outer = 0.5,
                    int offsetDegrees = 0,
                    bool flip = false)
    {
        create(cameraResolution,
               unwrappedResolution,
               centreX,
               centreY,
               inner,
               outer,
               offsetDegrees,
               flip);
    }

    OpenCVUnwrap360(const OpenCVUnwrap360 &unwrap)
    {
        m_CameraResolution = unwrap.m_CameraResolution;
        m_UnwrappedResolution = unwrap.m_UnwrappedResolution;
        m_CentrePixel = unwrap.m_CentrePixel;
        m_InnerPixel = unwrap.m_InnerPixel;
        m_OuterPixel = unwrap.m_OuterPixel;
        m_OffsetDegrees = unwrap.m_OffsetDegrees;
        m_Flip = unwrap.m_Flip;
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void create(const cv::Size &cameraResolution,
                const cv::Size &unwrappedResolution,
                double centreX = 0.5,
                double centreY = 0.5,
                double inner = 0.1,
                double outer = 0.5,
                int offsetDegrees = 0,
                bool flip = false)
    {
        // convert relative (0.0 to 1.0) to absolute pixel values
        m_CentrePixel = cv::Point(
                (int) round((double) cameraResolution.width * centreX),
                (int) round((double) cameraResolution.height * centreY));
        m_InnerPixel = (int) round((double) cameraResolution.height * inner);
        m_OuterPixel = (int) round((double) cameraResolution.height * outer);

        // Create x and y pixel maps
        m_UnwrapMapX.create(unwrappedResolution, CV_32FC1);
        m_UnwrapMapY.create(unwrappedResolution, CV_32FC1);

        // Save params
        m_CameraResolution = cameraResolution;
        m_UnwrappedResolution = unwrappedResolution;
        m_OffsetDegrees = offsetDegrees;
        m_Flip = flip;

        // Build unwrap maps
        updateMaps();
    }

    void updateMaps()
    {
        // Build unwrap maps
        const float pi = 3.141592653589793238462643383279502884f;
        float offsetRadians = m_OffsetDegrees * pi / 180.0f;
        for (int i = 0; i < m_UnwrappedResolution.height; i++) {
            for (int j = 0; j < m_UnwrappedResolution.width; j++) {
                // Get i as a fraction of unwrapped height, flipping if desired
                const float iFrac =
                        m_Flip ? 1.0f - ((float) i /
                                        (float) m_UnwrappedResolution.height)
                               : ((float) i /
                                  (float) m_UnwrappedResolution.height);

                // Convert i and j to polar
                const float r =
                        iFrac * (m_OuterPixel - m_InnerPixel) + m_InnerPixel;
                const float th =
                        (((float) j / (float) m_UnwrappedResolution.width) *
                         2.0f * pi) +
                        offsetRadians;

                // Remap onto sphere
                const float x = m_CentrePixel.x - r * sin(th);
                const float y = m_CentrePixel.y + r * cos(th);
                m_UnwrapMapX.at<float>(i, j) = x;
                m_UnwrapMapY.at<float>(i, j) = y;
            }
        }
    }

    void unwrap(const cv::Mat &input, cv::Mat &output)
    {
        cv::remap(input, output, m_UnwrapMapX, m_UnwrapMapY, cv::INTER_NEAREST);
    }

    /*
     * Serialise this object.
     */
    OpenCVUnwrap360 &operator>>(cv::FileStorage &fs)
    {
        // centre
        cv::Point2d centre = {
            (double) m_CentrePixel.x / (double) m_CameraResolution.width,
            (double) m_CentrePixel.y / (double) m_CameraResolution.height
        };
        fs << "centre" << centre;

        // radii
        fs << "inner"
           << (double) m_InnerPixel / (double) m_CameraResolution.height;
        fs << "outer"
           << (double) m_OuterPixel / (double) m_CameraResolution.height;

        // other
        fs << "offsetDegrees" << m_OffsetDegrees;
        fs << "flip" << m_Flip;

        return *this;
    }

    /*
     * Deserialise from a cv::FileStorage object (e.g. read from file).
     * 
     * **TODO**: Check that we are actually reading values from the file
     */
    cv::FileStorage &operator<<(cv::FileStorage &fs)
    {
        /*
         * We need to already know the camera resolution otherwise we won't be
         * able to convert the parameters from relative to absolute values.
         */
        assert(m_CameraResolution.width != 0 && m_CameraResolution.height != 0);
        assert(m_UnwrappedResolution.width != 0 && m_UnwrappedResolution.height != 0);
        
        // centre
        std::vector<double> centre(2);
        fs["centre"] >> centre;

        // inner and outer radius
        double inner = (double) fs["inner"];
        double outer = (double) fs["outer"];

        // other params
        int offsetDegrees = (int) fs["offsetDegrees"];
        bool flip;
        fs["flip"] >> flip;

        // create our unwrap maps
        create(m_CameraResolution,
               m_UnwrappedResolution,
               centre[0],
               centre[1],
               inner,
               outer,
               offsetDegrees,
               flip);
        
        return fs;
    }

    // Public members
    cv::Point m_CentrePixel;
    int m_InnerPixel, m_OuterPixel;
    int m_OffsetDegrees;
    bool m_Flip;
    
private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    cv::Size m_CameraResolution;
    cv::Size m_UnwrappedResolution;
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;
}; // OpenCVUnwrap360
}  // ImgProc
}  // GeNNRobotics
