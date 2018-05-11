#pragma once

#include "videoinput.h"
#include <opencv2/opencv.hpp>

namespace VideoIn {
class OpenCVInput
  : public cv::VideoCapture
  , public VideoInput
{
public:
    template<class T>
    OpenCVInput(T dev) : cv::VideoCapture(dev)
    {}

    bool readFrame(cv::Mat &outFrame)
    {
        (*this) >> outFrame;
        return outFrame.cols != 0;
    }
};
}
