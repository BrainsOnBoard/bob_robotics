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

// OpenCV
#include <opencv2/opencv.hpp>

// GeNN robotics includes
#include "../common/threadable.h"

// local includes
#include "input.h"

namespace GeNNRobotics {
namespace Video {
class Display : public Threadable
{
#define WINDOW_NAME "OpenCV display"

public:
    Display(Input &videoInput)
      : m_VideoInput(&videoInput)
    {}

    /*
     * Run the display on the main thread.
     */
    void run() override
    {
        // set opencv window to display full screen
        cvNamedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
        setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        cv::Mat frame;
        while (m_DoRun) {
            readNextFrame(frame);
            cv::imshow(WINDOW_NAME, frame);

            // quit when user presses esc
            if ((cv::waitKey(1) & 0xff) == 27) {
                break;
            }
        }
    }

protected:
    Input *m_VideoInput;

    virtual void readNextFrame(cv::Mat &frame)
    {
        if (!m_VideoInput->readFrame(frame)) {
            throw std::runtime_error("Error reading from video input");
        }
    }
}; // Display
} // Video
} // GeNNRobotics
