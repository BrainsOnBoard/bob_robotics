#pragma once

// BoB robotics includes
#include "input.h"
#include "common/macros.h"
#include "common/stopwatch.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <limits>
#include <random>

namespace BoBRobotics {
namespace Video {
//! A Video::Input which generates white noise, for testing purposes
template<typename GeneratorType = std::mt19937>
class RandomInput
  : public Input {
    using hertz_t = units::frequency::hertz_t;

public:
    RandomInput(const cv::Size &size, const std::string &cameraName = "random",
                const typename GeneratorType::result_type seed = getRandomSeed())
      : m_Size(size)
      , m_CameraName(cameraName)
      , m_Generator(seed)
	  , m_Distribution(0, 0xff)
    {}

    void setFrameRate(hertz_t fps)
    {
        BOB_ASSERT(fps > hertz_t{ 0 });
        m_FrameRate = fps;
        m_FrameDelay = 1.0 / fps;
        m_FrameRateTimer.start();
    }

    virtual hertz_t getFrameRate() const override
    {
        return m_FrameRate;
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        // The user can lower the framerate
        if (!isNewFrameReady()) {
            return false;
        }

        outFrame.create(m_Size, CV_8UC3);
        fillRandom(outFrame.data, outFrame.data + m_Size.area() * 3);
        return true;
    }

    virtual bool readGreyscaleFrame(cv::Mat &outFrame) override
    {
        // The user can lower the framerate
        if (!isNewFrameReady()) {
            return false;
        }

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
    std::uniform_int_distribution<int> m_Distribution;
    Stopwatch m_FrameRateTimer;
    hertz_t m_FrameRate{ std::numeric_limits<double>::infinity() };
    Stopwatch::Duration m_FrameDelay;

    void fillRandom(uchar *start, uchar *end)
    {
        std::generate(start, end, [this]()
            {
                return static_cast<uchar>(m_Distribution(m_Generator));
            });
    }

    bool isNewFrameReady()
    {
        if (isinf(m_FrameRate.value())) {
            return true;
        }

        if (m_FrameRateTimer.elapsed() > m_FrameDelay) {
            m_FrameRateTimer.start();
            return true;
        }
        return false;
    }

    static auto getRandomSeed()
    {
        std::random_device rd;
        return rd();
    }
};
} // Video
} // BoBRobotics
