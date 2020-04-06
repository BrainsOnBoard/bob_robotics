#pragma once

// Standard C++ includes
#include <string>

// OpenCV includes
#include <opencv2/core.hpp>

// Third-party includes
#include "third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::ImgProc::OpenCVUnwrap360
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace ImgProc {
using namespace units::literals;

class OpenCVUnwrap360
{
    using degree_t = units::angle::degree_t;

public:
    OpenCVUnwrap360();

    OpenCVUnwrap360(const cv::Size &cameraResolution,
                    const cv::Size &unwrappedResolution,
                    double centreX = 0.5,
                    double centreY = 0.5,
                    double inner = 0.1,
                    double outer = 0.5,
                    degree_t offsetAngle = 0.0_deg,
                    bool flip = false);

    OpenCVUnwrap360(const cv::Size &cameraResolution,
                    const cv::Size &unwrappedResolution,
                    const std::string &cameraName);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void create(const cv::Size &cameraResolution,
                const cv::Size &unwrappedResolution,
                double centreX = 0.5,
                double centreY = 0.5,
                double inner = 0.1,
                double outer = 0.5,
                degree_t offsetAngle = 0_deg,
                bool flip = false);

    void updateMaps();

    void unwrap(const cv::Mat &input, cv::Mat &output);

    //! Serialise this object.
    void write(cv::FileStorage &fs) const;

    /**!
     * \brief Deserialise from a cv::FileStorage object (e.g. read from file).
     *
     * **TODO**: Check that we are actually reading values from the file
     */
    void read(const cv::FileNode &node);

    // Public members
    cv::Point m_CentrePixel;
    int m_InnerPixel = 0, m_OuterPixel = 0;
    degree_t m_OffsetAngle = 0.0_deg;
    bool m_Flip = false;

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    cv::Size m_CameraResolution;
    cv::Size m_UnwrappedResolution;
    cv::Mat m_UnwrapMapX;
    cv::Mat m_UnwrapMapY;

    void createMaps();
}; // OpenCVUnwrap360

void
write(cv::FileStorage &fs,
      const std::string &,
      const OpenCVUnwrap360 &config);

void
read(const cv::FileNode &node,
     OpenCVUnwrap360 &x,
     OpenCVUnwrap360 defaultValue = OpenCVUnwrap360());

} // ImgProc
} // BoBRobotics

