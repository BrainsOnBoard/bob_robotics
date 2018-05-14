#pragma once

// Standard C includes
#include <cassert>
#define _USE_MATH_DEFINES
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
    {}

    OpenCVUnwrap360(const cv::Size &cameraResolution,
                    const cv::Size &unwrappedResolution,
                    double centreX = 0.5,
                    double centreY = 0.5,
                    double inner = 0.1,
                    double outer = 0.5,
                    int offsetDegrees = 0,
                    bool flip = false,
                    const std::string &filePath = "unknown_camera.yaml")
      : m_FilePath(filePath)
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
        create();
    }

    void create()
    {
        // Build unwrap maps
        double offsetRadians = m_OffsetDegrees * M_PI / 180.0;
        for (int i = 0; i < m_UnwrappedResolution.height; i++) {
            for (int j = 0; j < m_UnwrappedResolution.width; j++) {
                // Get i as a fraction of unwrapped height, flipping if desires
                const float iFrac =
                        m_Flip ? 1.0 - ((float) i /
                                        (float) m_UnwrappedResolution.height)
                               : ((float) i /
                                  (float) m_UnwrappedResolution.height);

                // Convert i and j to polar
                const float r =
                        iFrac * (m_OuterPixel - m_InnerPixel) + m_InnerPixel;
                const float th =
                        (((float) j / (float) m_UnwrappedResolution.width) *
                         2.0f * M_PI) +
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

    // Public members
    cv::Point m_CentrePixel;
    int m_InnerPixel, m_OuterPixel;
    int m_OffsetDegrees;
    bool m_Flip;

    // Begin static methods
    static OpenCVUnwrap360 *loadFromFile(const std::string &filePath,
                                         const cv::Size &cameraResolution)
    {
        // open YAML file
        cv::FileStorage fs(filePath, cv::FileStorage::READ);

        // resolution
        cv::Size unwrappedResolution;
        fs["unwrappedResolution"] >> unwrappedResolution;

        // centre
        std::vector<double> centre(2);
        fs["centre"] >> centre;

        // inner and outer radius
        double inner = (double) fs["inner"];
        double outer = (double) fs["outer"];

        // other params
        bool flip;
        fs["flip"] >> flip;
        int offsetDegrees = (int) fs["offsetDegrees"];

        // close YAML file
        fs.release();

        return new OpenCVUnwrap360(cameraResolution,
                                   unwrappedResolution,
                                   centre[0],
                                   centre[1],
                                   inner,
                                   outer,
                                   offsetDegrees,
                                   flip);
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    std::string m_FilePath;
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;
    cv::Size m_CameraResolution, m_UnwrappedResolution;
};