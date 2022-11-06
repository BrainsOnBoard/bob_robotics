// BoB robotics includes
#include "os/keycodes.h"
#include "video/display.h"

// Standard C++ includes
#include <chrono>
#include <thread>

using namespace std::literals;

namespace BoBRobotics {
namespace Video {

Display::Display(Input &videoInput, const bool fullScreen)
  : m_VideoInput(videoInput)
{
    // set opencv window to display full screen
    if (fullScreen) {
        cv::namedWindow(WindowName, cv::WINDOW_NORMAL);
        setWindowProperty(WindowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
        cv::namedWindow(WindowName, cv::WINDOW_AUTOSIZE);
    }
}

Display::Display(Input &videoInput, const cv::Size &unwrapRes, const bool fullScreen)
  : Display(videoInput, fullScreen)
{
    if (videoInput.needsUnwrapping()) {
        m_ShowUnwrapped = true;
        auto unwrapper = videoInput.createUnwrapper(unwrapRes);
        m_Unwrapper = std::make_unique<ImgProc::OpenCVUnwrap360>(std::move(unwrapper));
    }
}

Display::~Display()
{
    close();
}

Video::Input &Display::getVideoInput()
{
    return m_VideoInput;
}

bool Display::isOpen()
{
    return m_IsOpen;
}

bool Display::update(cv::Mat &frame)
{
    //cv::Mat frame;
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

void Display::close()
{
    stop();
    if (isOpen()) {
        cv::destroyWindow(WindowName);
        m_IsOpen = false;
    }
}

bool Display::readFrame(cv::Mat &frame)
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

void Display::runInternal()
{
    while (isRunning()) {
        // check for a new frame
        cv::Mat frame;
        if (!update(frame)) {
            std::this_thread::sleep_for(25ms);
        }
    }
}

} // Video
} // BoBRobotics
