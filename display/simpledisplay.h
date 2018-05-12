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

#include "../videoin/videoinput.h"
#include <opencv2/opencv.hpp>

#include <memory>
#include <stdexcept>
#include <thread>

namespace Display {
class SimpleDisplay
{
public:
    /*
     * We use a shared_ptr to make sure that the
     * camera object will not be destroyed before this object has finished with
     * it.
     */
    SimpleDisplay(std::shared_ptr<VideoIn::VideoInput> vid)
      : m_VideoInput(vid)
    {}

    ~SimpleDisplay()
    {
        close();
    }

    /*
     * Quit display and stop background thread properly if needed.
     */
    void close()
    {
        m_Running = false;
        joinThread();
    }

    /*
     * Run the display on the main thread.
     */
    void run()
    {
        // set opencv window to display full screen
        cvNamedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
        setWindowProperty(
                WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        cv::Mat frame;
        while (m_Running) {
            if (!m_VideoInput->readFrame(frame)) {
                throw std::runtime_error("Error reading from video input");
            }

            cv::imshow(WINDOW_NAME, frame);

            // quit when user presses esc
            if ((cv::waitKey(1) & 0xff) == 27) {
                break;
            }
        }
    }

    /*
     * Run the display on a background thread.
     */
    void startThread()
    {
        m_DisplayThread =
                std::unique_ptr<std::thread>(new std::thread(runThread, this));
    }

    /*
     * Join the background thread.
     */
    void joinThread()
    {
        if (m_DisplayThread) {
            m_DisplayThread->join();
        }
    }

private:
    std::shared_ptr<VideoIn::VideoInput> m_VideoInput;
    std::unique_ptr<std::thread> m_DisplayThread;
    bool m_Running = true;

    static constexpr const char *WINDOW_NAME = "OpenCV display";

    /*
     * Wrapper function needed because one cannot pass an object's method to a
     * thread object.
     */
    static void runThread(void *userData)
    {
        reinterpret_cast<SimpleDisplay *>(userData)->run();
    }
};
}
