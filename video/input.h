#pragma once

// BoBRobotics includes
#include "../imgproc/opencv_unwrap_360.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <stdexcept>
#include <string>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace BoBRobotics {
namespace Video {
using namespace std::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Video::Input
//----------------------------------------------------------------------------
//! An abstract class representing a video input stream
class Input
{
public:
    virtual ~Input()
    {}

    /*!
     * \brief Create an ImgProc::OpenCVUnwrap360 object for this video stream
     *
     * @param unwrapRes The resolution of the unwrapped image
     */
    ImgProc::OpenCVUnwrap360 createUnwrapper(const cv::Size &unwrapRes)
    {
        // Create unwrapper and return
        return ImgProc::OpenCVUnwrap360(getOutputSize(), unwrapRes,
                                        getCameraName());
    }

    /*!
     * \brief Get the name of this type of camera as a (short) string
     *
     * Note that this is used to load the appropriate unwrapping parameters
     * (we look for a file called [camera name].yaml).
     */
    virtual std::string getCameraName() const
    {
        return DefaultCameraName;
    }

    /*!
     * \brief Try to read a frame in greyscale from this video source
     *
     * @return Whether a new frame was read
     */
    virtual bool readGreyscaleFrame(cv::Mat &outFrame)
    {
        // If reading (colour frame) was successful
        if(readFrame(m_IntermediateFrame)) {
            // Make sure frame is of right size and type
            outFrame.create(m_IntermediateFrame.size(), CV_8UC1);

            // Convert intermediate frame to greyscale
            cv::cvtColor(m_IntermediateFrame, outFrame, cv::COLOR_BGR2GRAY);
            return true;
        }
        else {
            return false;
        }
    }

    //! Whether this video source needs unwrapping with an ImgProc::OpenCVUnwrap360
    virtual bool needsUnwrapping() const
    {
        // only panoramic cameras are defined with the camera name specified
        return getCameraName() != DefaultCameraName;
    }

    //! Set the output resolution of this video stream
    virtual void setOutputSize(const cv::Size &)
    {
        throw std::runtime_error("This camera's resolution cannot be changed at runtime");
    }

    //! Get the current output resolution of this video stream
    virtual cv::Size getOutputSize() const = 0;

    /*!
     * \brief Try to read a frame in colour from this video source
     *
     * @return Whether a new frame was read
     */
    virtual bool readFrame(cv::Mat &outFrame) = 0;

    //! Read a frame synchronously, blocking until a new frame is received
    void readFrameSync(cv::Mat &outFrame)
    {
        while (!readFrame(outFrame)) {
            std::this_thread::sleep_for(10ms);
        }
    }

    //! Allows OpenCV to serialise info about this Input
    void write(cv::FileStorage& fs) const
    {
        fs << "{";
        fs << "name" << getCameraName();
        fs << "resolution" << getOutputSize();
        fs << "isPanoramic" << needsUnwrapping();
        fs << "}";
    }

    static constexpr const char *DefaultCameraName = "unknown_camera";
private:
    cv::Mat m_IntermediateFrame;
}; // Input

//! More OpenCV boilerplate
inline void write(cv::FileStorage &fs, const std::string &, const Input &camera)
{
    camera.write(fs);
}
} // Video
} // BoBRobotics
