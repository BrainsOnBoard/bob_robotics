// Standard C++ includes
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/threadable.h"
#include "../imgproc/opencv_unwrap_360.h"
#include "../os/keycodes.h"

// Local includes
#include "input.h"

namespace BoBRobotics {
namespace Video {
using namespace std::literals;

//----------------------------------------------------------------------------
// BoBRobotics::Video::Display
//----------------------------------------------------------------------------
/*!
 * \brief Display a video source on screen
 *
 * This is a simple class for displaying a VideoInput (e.g. a webcam) on screen.
 * An example of its use is given in examples/display_test.
 *
 * You can optionally run the display on a separate thread by invoking the
 * runInBackground() method.
 *
 * Should be built with OpenCV and -pthread.
 */
class Display : public Threadable
{
#define WINDOW_NAME "OpenCV display"

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
    {}

    //! Close display window and destroy this object
    virtual ~Display()
    {
        close();
    }

    /*!
     * \brief Create a new display with unwrapping enabled if the videoInput supports it
     *
     * @param videoInput The video source to display
     * @param unwrapRes The size of the target image after unwrapping
     * @param fullScreen Whether or not to display fullscreen
     */
    Display(Input &videoInput, const cv::Size &unwrapRes, const bool fullScreen = false)
      : m_VideoInput(videoInput)
      , m_FullScreen(fullScreen)
    {
        if (videoInput.needsUnwrapping()) {
            m_ShowUnwrapped = true;
            auto unwrapper = videoInput.createUnwrapper(unwrapRes);
            m_Unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(std::move(unwrapper));
        }
    }

    //! Return true if the display window is open
    bool isOpen() const
    {
        return m_Open;
    }

    /*!
     * \brief Try to read a new frame from the video source and display it
     *
     * \return Whether a new frame was successfully read
     */
    bool update()
    {
        if (!m_Open) {
            // set opencv window to display full screen
            cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
            if (m_FullScreen) {
                setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            }
            m_Open = true;
        }

        cv::Mat frame;
        bool newFrame = readFrame(frame);
        if (newFrame) {
            // display the frame on screen
            cv::imshow(WINDOW_NAME, frame);
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
        if (m_Open) {
            cv::destroyWindow(WINDOW_NAME);
            m_Open = false;
        }
    }

protected:
    bool m_Open = false;
    cv::Mat m_Frame, m_Unwrapped;
    bool m_ShowUnwrapped = false;
    std::unique_ptr<ImgProc::OpenCVUnwrap360> m_Unwrapper;
    Input &m_VideoInput;
    bool m_FullScreen;

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
}; // Display
} // Video
} // BoBRobotics
