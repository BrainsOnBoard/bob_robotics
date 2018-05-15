#pragma once

#include <opencv2/opencv.hpp>

namespace VideoIn {
class VideoInput
{
public:
    virtual ~VideoInput()
    {}
    virtual void setOutputSize(const cv::Size &outSize)
    {}
    virtual const std::string getDefaultUnwrapParams()
    {
        return "";
    }

    virtual bool readFrame(cv::Mat &outFrame) = 0;
    virtual const std::string getCameraName() = 0;
};
}
