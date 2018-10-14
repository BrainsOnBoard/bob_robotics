#pragma once

// BoB robotics includes
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <random>

namespace BoBRobotics {
namespace Video {
//! A Video::Input which generates white noise, for testing purposes
template<typename GeneratorType = std::mt19937>
class RandomInput : public Input {
public:
    RandomInput(const cv::Size &size, const std::string &cameraName = "random",
                const typename GeneratorType::result_type seed = getRandomSeed())
      : m_Size(size)
      , m_CameraName(cameraName)
      , m_Generator(seed)
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

    virtual std::string getCameraName() const override
    {
        return m_CameraName;
    }

    virtual cv::Size getOutputSize() const override
    {
        return m_Size;
    }

    virtual void setOutputSize(const cv::Size &size) override
    {
        m_Size = size;
    }

    virtual bool needsUnwrapping() const override
    {
        /*
         * If the user sets the camera name to something else, it's probably
         * because they want to simulate a panoramic camera.
         */
        return m_CameraName != "random";
    }

private:
    cv::Size m_Size;
    std::string m_CameraName;
    GeneratorType m_Generator;
    std::uniform_int_distribution<uchar> m_Distribution;

    void fillRandom(uchar *start, uchar *end)
    {
        std::generate(start, end, [this](){ return m_Distribution(m_Generator); });
    }

    static auto getRandomSeed()
    {
        std::random_device rd;
        return rd();
    }
};
} // Video
} // BoBRobotics
