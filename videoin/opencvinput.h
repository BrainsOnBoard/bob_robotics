#pragma once

#include "videoinput.h"
#include <opencv2/opencv.hpp>

namespace VideoIn {
class OpenCVInput
  : public cv::VideoCapture
  , public VideoInput
{
public:
    OpenCVInput()
      : OpenCVInput(0)
    {}

    template<class T>
    OpenCVInput(T dev)
      : cv::VideoCapture(dev)
    {}

    bool readFrame(cv::Mat &outFrame)
    {
        (*this) >> outFrame;
        return outFrame.cols != 0;
    }

    void setOutputSize(cv::Size outSize)
    {
        set(cv::CAP_PROP_FRAME_WIDTH, outSize.width);
        set(cv::CAP_PROP_FRAME_HEIGHT, outSize.height);
    }
};
}
