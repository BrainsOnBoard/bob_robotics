#pragma once

// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Video {

//----------------------------------------------------------------------------
// BoBRobotics::Video::Input
//----------------------------------------------------------------------------
//! An abstract class representing a video input stream
class Input
{
public:
    virtual ~Input();

    /*!
     * \brief Create an ImgProc::OpenCVUnwrap360 object for this video stream
     *
     * @param unwrapRes The resolution of the unwrapped image
     */
    ImgProc::OpenCVUnwrap360 createUnwrapper(const cv::Size &unwrapRes) const;

    /*!
     * \brief Get the name of this type of camera as a (short) string
     *
     * Note that this is used to load the appropriate unwrapping parameters
     * (we look for a file called [camera name].yaml).
     */
    virtual std::string getCameraName() const;

    /*!
     * \brief Try to read a frame in greyscale from this video source
     *
     * @return Whether a new frame was read
     */
    virtual bool readGreyscaleFrame(cv::Mat &outFrame);

    //! Whether this video source needs unwrapping with an ImgProc::OpenCVUnwrap360
    virtual bool needsUnwrapping() const;

    //! Set the output resolution of this video stream
    virtual void setOutputSize(const cv::Size &);

    //! Get the current output resolution of this video stream
    virtual cv::Size getOutputSize() const = 0;

    /*!
     * \brief Try to read a frame in colour from this video source
     *
     * @return Whether a new frame was read
     */
    virtual bool readFrame(cv::Mat &outFrame) = 0;

    //! Read a frame synchronously, blocking until a new frame is received
    void readFrameSync(cv::Mat &outFrame);

    //! Allows OpenCV to serialise info about this Input
    void write(cv::FileStorage &fs) const;

    static constexpr const char *DefaultCameraName = "unknown_camera";

private:
    cv::Mat m_IntermediateFrame;
}; // Input

//! More OpenCV boilerplate
void
write(cv::FileStorage &fs, const std::string &, const Input &camera);
} // Video
} // BoBRobotics
