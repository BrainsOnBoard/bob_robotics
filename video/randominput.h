#pragma once

// BoB robotics includes
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <random>

namespace BoBRobotics {
namespace Video {
//! A Video::Input which generates white noise, for testing purposes
class RandomInput : public Input {
public:
    RandomInput(const cv::Size &size, const std::string &cameraName = "random")
      : m_Size(size)
      , m_CameraName(cameraName)
      , m_Generator(m_RandomDevice())
    {}

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        outFrame.create(m_Size, CV_8UC3);
        fillRandom(outFrame.data, outFrame.data + m_Size.area() * 3);
        return true;
    }

    virtual bool readGreyscaleFrame(cv::Mat &outFrame) override
    {
        outFrame.create(m_Size, CV_8UC1);
        fillRandom(outFrame.data, outFrame.data + m_Size.area());
        return true;
    }

    virtual std::string getCameraName() const
    {
        return m_CameraName;
    }

    virtual cv::Size getOutputSize() const
    {
        return m_Size;
    }

    virtual void setOutputSize(const cv::Size &size)
    {
        m_Size = size;
    }

private:
    cv::Size m_Size;
    std::string m_CameraName;
    std::random_device m_RandomDevice;
    std::mt19937 m_Generator;
    std::uniform_int_distribution<uchar> m_Distribution;

    void fillRandom(uchar *start, uchar *end)
    {
        for (uchar *p = start; p < end; p++) {
            *p = m_Distribution(m_Generator);
        }
    }
};
} // Video
} // BoBRobotics
