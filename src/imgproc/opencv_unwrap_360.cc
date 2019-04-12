// BoB robotics includes
#include "common/assert.h"
#include "common/logging.h"
#include "imgproc/opencv_unwrap_360.h"

// Standard C++ includes
#include <stdexcept>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::ImgProc::OpenCVUnwrap360
//----------------------------------------------------------------------------
namespace BoBRobotics {
namespace ImgProc {
using namespace units::literals;
using namespace units::angle;

OpenCVUnwrap360::OpenCVUnwrap360()
{}

OpenCVUnwrap360::OpenCVUnwrap360(const cv::Size &cameraResolution,
                                 const cv::Size &unwrappedResolution,
                                 double centreX,
                                 double centreY,
                                 double inner,
                                 double outer,
                                 degree_t offsetAngle,
                                 bool flip)
{
    create(cameraResolution,
           unwrappedResolution,
           centreX,
           centreY,
           inner,
           outer,
           offsetAngle,
           flip);
}

OpenCVUnwrap360::OpenCVUnwrap360(const cv::Size &cameraResolution,
                                 const cv::Size &unwrappedResolution,
                                 const std::string &cameraName)
  : m_CameraResolution(cameraResolution)
  , m_UnwrappedResolution(unwrappedResolution)
{
    const std::string fileName = cameraName + ".yaml";
    filesystem::path filePath(fileName);

    // first check if file exists in working directory
    if (!filePath.exists()) {
        // next check if there is a local bob_robotics folder (i.e. git
        // submodule)
        const filesystem::path paramsDir = filesystem::path("include") /
                                           filesystem::path("imgproc") / "unwrapparams";

        filePath = filesystem::path("bob_robotics") / paramsDir / fileName;
        if (!filePath.exists()) {
            // lastly look for environment variable pointing to
            // bob_robotics
            static const char *envVarName = "BOB_ROBOTICS_PATH";
            const char *env = std::getenv(envVarName);
            if (!env) {
                throw std::runtime_error(std::string(envVarName) +
                                         " environment variable is not set and unwrap "
                                         "parameters file could not be found locally");
            }

            filePath = filesystem::path(env) / paramsDir / fileName;
            if (!filePath.exists()) {
                throw std::runtime_error(
                        "Could not find unwrap parameters file for " +
                        cameraName);
            }
        }
    }

    // read unwrap parameters from file
    LOG_INFO << "Loading unwrap parameters from " << filePath.str();
    cv::FileStorage fs(filePath.str(), cv::FileStorage::READ);
    read(fs["unwrapper"]);
    fs.release();
}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
void
OpenCVUnwrap360::create(const cv::Size &cameraResolution,
                        const cv::Size &unwrappedResolution,
                        double centreX,
                        double centreY,
                        double inner,
                        double outer,
                        degree_t offsetAngle,
                        bool flip)
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
    m_OffsetAngle = offsetAngle;
    m_Flip = flip;

    // Build unwrap maps
    createMaps();
}

void
OpenCVUnwrap360::updateMaps()
{
    // Build unwrap maps
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
            const degree_t th =
                    (((double) j / (double) m_UnwrappedResolution.width) *
                     360.0_deg) +
                    m_OffsetAngle;

            // Remap onto sphere
            const float x = m_CentrePixel.x - r * units::math::sin(th);
            const float y = m_CentrePixel.y + r * units::math::cos(th);
            m_UnwrapMapX.at<float>(i, j) = x;
            m_UnwrapMapY.at<float>(i, j) = y;
        }
    }
}

void
OpenCVUnwrap360::unwrap(const cv::Mat &input, cv::Mat &output)
{
    cv::remap(input, output, m_UnwrapMapX, m_UnwrapMapY, cv::INTER_NEAREST);
}

void
OpenCVUnwrap360::write(cv::FileStorage &fs) const
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
    fs << "offsetDegrees" << m_OffsetAngle.value();
    fs << "flip" << m_Flip;
    fs << "}";
}

void
OpenCVUnwrap360::read(const cv::FileNode &node)
{
    /*
         * We need to already know the camera resolution otherwise we won't be
         * able to convert the parameters from relative to absolute values.
         */
    BOB_ASSERT(m_CameraResolution.width != 0 && m_CameraResolution.height != 0);
    BOB_ASSERT(m_UnwrappedResolution.width != 0 && m_UnwrappedResolution.height != 0);

    // centre
    std::vector<double> centre(2);
    node["centre"] >> centre;

    // inner and outer radius
    double inner = (double) node["inner"];
    double outer = (double) node["outer"];

    // other params
    degree_t offsetAngle = units::make_unit<degree_t>((double) node["offsetDegrees"]);
    bool flip;
    node["flip"] >> flip;

    // create our unwrap maps
    create(m_CameraResolution,
           m_UnwrappedResolution,
           centre[0],
           centre[1],
           inner,
           outer,
           offsetAngle,
           flip);
}

void
OpenCVUnwrap360::createMaps()
{
    m_UnwrapMapX.create(m_UnwrappedResolution, CV_32FC1);
    m_UnwrapMapY.create(m_UnwrappedResolution, CV_32FC1);
    updateMaps();
}

void
write(cv::FileStorage &fs,
      const std::string &,
      const OpenCVUnwrap360 &config)
{
    config.write(fs);
}

void
read(const cv::FileNode &node,
     OpenCVUnwrap360 &x,
     OpenCVUnwrap360 defaultValue)
{
    if (node.empty()) {
        x = defaultValue;
    } else {
        x.read(node);
    }
}

} // ImgProc
} // BoBRobotics
