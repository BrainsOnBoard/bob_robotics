#pragma once

// BoB robotics includes
#include "../common/threadable.h"
#include "../imgproc/opencv_unwrap_360.h"
#include "../os/keycodes.h"
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace BoBRobotics {
namespace Video {
using namespace std::literals;

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
class Display : public Threadable
{
public:
    /*!
     * \brief Create a new display with unwrapping disabled
     *
     * @param videoInput The video source to display
     * @param fullScreen Whether or not to display fullscreen
     */
    Display(Input &videoInput, const bool fullScreen = false)
      : m_VideoInput(videoInput)
      , m_FullScreen(fullScreen)
    {
        // set opencv window to display full screen
        cv::namedWindow(WindowName, cv::WINDOW_AUTOSIZE);
        if (fullScreen) {
            setWindowProperty(WindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        }
    }


    /*!
     * \brief Create a new display with unwrapping enabled if the videoInput supports it
     *
     * @param videoInput The video source to display
     * @param unwrapRes The size of the target image after unwrapping
     * @param fullScreen Whether or not to display fullscreen
     */
    Display(Input &videoInput, const cv::Size &unwrapRes, const bool fullScreen = false)
      : Display(videoInput, fullScreen)
    {
        if (videoInput.needsUnwrapping()) {
            m_ShowUnwrapped = true;
            auto unwrapper = videoInput.createUnwrapper(unwrapRes);
            m_Unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(std::move(unwrapper));
        }
    }

    //! Close display window and destroy this object
    virtual ~Display()
    {
        close();
    }

    //! Return true if the display window is open
    bool isOpen() const
    {
        return cvGetWindowHandle(WindowName) != nullptr;
    }

    /*!
     * \brief Try to read a new frame from the video source and display it
     *
     * \return Whether a new frame was successfully read
     */
    bool update()
    {
        cv::Mat frame;
        bool newFrame = readFrame(frame);
        if (newFrame) {
            // display the frame on screen
            cv::imshow(WindowName, frame);
        }

        // get keyboard input
        switch (cv::waitKeyEx(1) & OS::KeyMask) {
        case 'u': // toggle unwrapping
            m_ShowUnwrapped = !m_ShowUnwrapped;
            break;
        case OS::KeyCodes::Escape:
            close();
        }

        return newFrame;
    }

    //! Close the display and stop the background thread if needed
    virtual void close()
    {
        if (isOpen()) {
            cv::destroyWindow(WindowName);
        }
    }

protected:
    virtual bool readFrame(cv::Mat &frame)
    {
        if (!m_VideoInput.readFrame(m_Frame)) {
            return false;
        }

        // unwrap frame if required
        if (m_Unwrapper && m_ShowUnwrapped) {
            m_Unwrapper->unwrap(m_Frame, m_Unwrapped);
            frame = m_Unwrapped;
        } else {
            frame = m_Frame;
        }
        return true;
    }

    virtual void runInternal() override
    {
        while (isRunning()) {
            // check for a new frame
            if (!update()) {
                std::this_thread::sleep_for(25ms);
            }
        }
    }

private:
    cv::Mat m_Frame, m_Unwrapped;
    bool m_ShowUnwrapped = false;
    std::unique_ptr<ImgProc::OpenCVUnwrap360> m_Unwrapper;
    Input &m_VideoInput;
    bool m_FullScreen;
    static constexpr const char *WindowName = "BoB robotics display";
}; // Display
} // Video
} // BoBRobotics
