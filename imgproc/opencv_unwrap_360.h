#pragma once

// Standard C includes
#include <cassert>
#include <cmath>

// OpenCV includes
#include "../common/opencv.h"

//----------------------------------------------------------------------------
// BoBRobotics::ImgProc::OpenCVUnwrap360
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace ImgProc {
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

        // Save params
        m_CameraResolution = cameraResolution;
        m_UnwrappedResolution = unwrappedResolution;
        m_OffsetDegrees = offsetDegrees;
        m_Flip = flip;

        // Build unwrap maps
        createMaps();
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
    void write(cv::FileStorage& fs) const
    {
        fs << "{";
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
        fs << "}";
    }

    /*
     * Deserialise from a cv::FileStorage object (e.g. read from file).
     * 
     * **TODO**: Check that we are actually reading values from the file
     */
    void read(const cv::FileNode &node)
    {
        /*
         * We need to already know the camera resolution otherwise we won't be
         * able to convert the parameters from relative to absolute values.
         */
        assert(m_CameraResolution.width != 0 && m_CameraResolution.height != 0);
        assert(m_UnwrappedResolution.width != 0 && m_UnwrappedResolution.height != 0);

        // centre
        std::vector<double> centre(2);
        node["centre"] >> centre;

        // inner and outer radius
        double inner = (double) node["inner"];
        double outer = (double) node["outer"];

        // other params
        int offsetDegrees = (int) node["offsetDegrees"];
        bool flip;
        node["flip"] >> flip;

        // create our unwrap maps
        create(m_CameraResolution,
               m_UnwrappedResolution,
               centre[0],
               centre[1],
               inner,
               outer,
               offsetDegrees,
               flip);
    }

    // Public members
    cv::Point m_CentrePixel;
    int m_InnerPixel = 0, m_OuterPixel = 0;
    int m_OffsetDegrees = 0;
    bool m_Flip = false;

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    cv::Size m_CameraResolution;
    cv::Size m_UnwrappedResolution;
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;

    void createMaps()
    {
        m_UnwrapMapX.create(m_UnwrappedResolution, CV_32FC1);
        m_UnwrapMapY.create(m_UnwrappedResolution, CV_32FC1);
        updateMaps();
    }
}; // OpenCVUnwrap360

inline void write(cv::FileStorage &fs, const std::string&, const OpenCVUnwrap360 &config)
{
    config.write(fs);
}

inline void read(const cv::FileNode &node, OpenCVUnwrap360 &x, OpenCVUnwrap360 defaultValue = OpenCVUnwrap360())
{
    if(node.empty()) {
        x = defaultValue;
    }
    else {
        x.read(node);
    }
}


}  // ImgProc
}  // BoBRobotics
