/*
 * This is a simple class for displaying a VideoInput (e.g. a webcam) on screen.
 * An example of its use is given in simpledisplay_test.cc (build with make).
 * The user can quit by pressing escape.
 *
 * You can optionally run the display on a separate thread by invoking the
 * startThread() method.
 *
 * Should be built with OpenCV and -pthread.
 */

// C++ includes
#include <stdexcept>

// opencv
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "input.h"

namespace Video {
class SimpleDisplay
{
public:
    virtual ~SimpleDisplay()
    {
        close();
    }

    /*
     * Quit display and stop background thread properly if needed.
     */
    void close()
    {
        m_Running = false;
    }

    /*
     * Gets the next frame from the video stream.
     */
    virtual void getNextFrame(Input &videoInput, cv::Mat &outFrame)
    {
        if (!videoInput.readFrame(outFrame)) {
            throw std::runtime_error("Error reading from video input");
        }
    }

    /*
     * Run the display on the main thread.
     */
    void run(Input &videoInput)
    {
        // set opencv window to display full screen
        cvNamedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
        setWindowProperty(
                WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        cv::Mat frame;
        while (m_Running) {
            getNextFrame(videoInput, frame);
            cv::imshow(WINDOW_NAME, frame);

            // quit when user presses esc
            if ((cv::waitKey(1) & 0xff) == 27) {
                break;
            }
        }
    }

private:
    bool m_Running = true;
    static constexpr const char *WINDOW_NAME = "OpenCV display";
};
}
