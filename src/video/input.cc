// BoB robotics includes
#include "plog/Log.h"
#include "common/macros.h"
#include "video/input.h"

// Standard C++ includes
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace std::literals;

namespace BoBRobotics {
namespace Video {

Input::~Input()
{
    LOG_DEBUG << "Camera closed";
}

ImgProc::OpenCVUnwrap360
Input::createUnwrapper(const cv::Size &unwrapRes) const
{
    // Create unwrapper and return
    return ImgProc::OpenCVUnwrap360(getOutputSize(), unwrapRes, getCameraName());
}

std::string
Input::getCameraName() const
{
    return DefaultCameraName;
}

BOB_NOT_IMPLEMENTED(units::frequency::hertz_t Input::getFrameRate() const)

bool
Input::readGreyscaleFrame(cv::Mat &outFrame)
{
    // If reading (colour frame) was successful
    if (readFrame(m_IntermediateFrame)) {
        // Make sure frame is of right size and type
        outFrame.create(m_IntermediateFrame.size(), CV_8UC1);

        // Convert intermediate frame to greyscale
        cv::cvtColor(m_IntermediateFrame, outFrame, cv::COLOR_BGR2GRAY);
        return true;
    } else {
        return false;
    }
}

bool
Input::needsUnwrapping() const
{
    // only panoramic cameras are defined with the camera name specified
    return getCameraName() != DefaultCameraName;
}

BOB_NOT_IMPLEMENTED(void Input::setOutputSize(const cv::Size &))

void
Input::readFrameSync(cv::Mat &outFrame)
{
    while (!readFrame(outFrame)) {
        std::this_thread::sleep_for(5ms);
    }
}

void
Input::readGreyscaleFrameSync(cv::Mat &outFrame)
{
    while (!readGreyscaleFrame(outFrame)) {
        std::this_thread::sleep_for(5ms);
    }
}

void
Input::write(cv::FileStorage &fs) const
{
    fs << "{";
    fs << "name" << getCameraName();
    fs << "resolution" << getOutputSize();
    fs << "isPanoramic" << needsUnwrapping();
    fs << "}";
}

void write(cv::FileStorage &fs, const std::string &, const Input &camera)
{
    camera.write(fs);
}
} // Video
} // BoBRobotics
