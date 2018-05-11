#pragma once

#include <opencv2/opencv.hpp>

namespace VideoIn {
class VideoInput
{
public:
    virtual bool readFrame(cv::Mat &outFrame) = 0;
    virtual void setOutputSize(cv::Size outSize)
    {}
};
}
