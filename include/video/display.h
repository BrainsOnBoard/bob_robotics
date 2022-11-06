#pragma once

// BoB robotics includes
#include "common/threadable.h"
#include "imgproc/opencv_unwrap_360.h"
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <memory>

namespace BoBRobotics {
namespace Video {

//----------------------------------------------------------------------------
// BoBRobotics::Video::Display
//----------------------------------------------------------------------------
/*!
 * \brief Display a video source on screen
 *
 * This is a simple class for displaying a Video::Input (e.g. a webcam) on screen.
 * An example of its use is given in examples/display.
 *
 * You can optionally run the display on a separate thread by invoking the
 * runInBackground() method.
 *
 * Should be built with OpenCV and -pthread.
 */
class Display final
  : public Threadable
{
public:
    /*!
     * \brief Create a new display with unwrapping disabled
     *
     * @param videoInput The video source to display
     * @param fullScreen Whether or not to display fullscreen
     */
    Display(Input &videoInput, const bool fullScreen = false);

    /*!
     * \brief Create a new display with unwrapping enabled if the videoInput supports it
     *
     * @param videoInput The video source to display
     * @param unwrapRes The size of the target image after unwrapping
     * @param fullScreen Whether or not to display fullscreen
     */
    Display(Input &videoInput,
            const cv::Size &unwrapRes,
            const bool fullScreen = false);

    //! Close display window and destroy this object
    virtual ~Display() override;

    //! Gets the Video::Input object this Display is reading from
    Video::Input &getVideoInput();

    //! Return true if the display window is open
    bool isOpen();

    /*!
     * \brief Try to read a new frame from the video source and display it
     *
     * \return Whether a new frame was successfully read
     */
    bool update(cv::Mat &frame);

    //! Close the display and stop the background thread if needed
    virtual void close();

protected:
    virtual bool readFrame(cv::Mat &frame);

    virtual void runInternal() override;

private:
    Input &m_VideoInput;
    cv::Mat m_Frame, m_Unwrapped;
    std::unique_ptr<ImgProc::OpenCVUnwrap360> m_Unwrapper;
    static constexpr const char *WindowName = "BoB robotics display";
    bool m_ShowUnwrapped = false, m_IsOpen = true;
}; // Display
} // Video
} // BoBRobotics
