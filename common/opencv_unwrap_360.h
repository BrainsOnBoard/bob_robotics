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
        const float pi = 3.141592653589793238462643383279502884f;
        double offsetRadians = m_OffsetDegrees * pi / 180.0;
        for (int i = 0; i < m_UnwrappedResolution.height; i++) {
            for (int j = 0; j < m_UnwrappedResolution.width; j++) {
                // Get i as a fraction of unwrapped height, flipping if desired
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

    void writeFile()
    {
        std::cout << "Writing to " << m_FilePath << "..." << std::endl;
        cv::FileStorage fs(m_FilePath, cv::FileStorage::WRITE);

        // resolution
        fs << "unwrappedResolution" << m_UnwrappedResolution;

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

        // close file
        fs.release();
    }

    // Public members
    cv::Point m_CentrePixel;
    int m_InnerPixel, m_OuterPixel;
    int m_OffsetDegrees;
    bool m_Flip;
    cv::Size m_CameraResolution, m_UnwrappedResolution;

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
        int offsetDegrees = (int) fs["offsetDegrees"];
        bool flip;
        fs["flip"] >> flip;

        // close YAML file
        fs.release();

        // create new unwrapper on the heap - user is responsible for deleting
        // it
        return new OpenCVUnwrap360(cameraResolution,
                                   unwrappedResolution,
                                   centre[0],
                                   centre[1],
                                   inner,
                                   outer,
                                   offsetDegrees,
                                   flip,
                                   filePath);
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    std::string m_FilePath;
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;
};