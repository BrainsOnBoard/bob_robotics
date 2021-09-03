#pragma once

// BoB robotics includes
#include "common/stopwatch.h"
#include "video/input.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <limits>
#include <random>
#include <thread>

namespace BoBRobotics {
namespace Video {
//! A Video::Input which generates white noise, for testing purposes
template<typename GeneratorType = std::mt19937>
class RandomInput : public Input {
    using hertz_t = units::frequency::hertz_t;

public:
    RandomInput(const cv::Size &size,
                typename GeneratorType::result_type seed = std::random_device{}())
      : m_Size(size)
      , m_FrameRate(std::numeric_limits<double>::infinity())
      , m_CameraName("random")
      , m_Generator(seed)
	  , m_Distribution(0, 0xff)
    {
        m_FrameTimer.start();
    }

    virtual bool readFrame(cv::Mat &outFrame) override
    {
        if (!frameReady()) {
            return false;
        }

        outFrame.create(m_Size, CV_8UC3);
        fillRandom(outFrame.data, outFrame.data + m_Size.area() * 3);
        return true;
    }

    virtual bool readGreyscaleFrame(cv::Mat &outFrame) override
    {
        if (!frameReady()) {
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

    virtual hertz_t getFrameRate() const override
    {
        return m_FrameRate;
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

    void setCameraName(std::string name)
    {
        m_CameraName = std::move(name);
    }

    void setFrameRate(hertz_t frameRate)
    {
        m_FrameRate = frameRate;
    }

private:
    cv::Size m_Size;
    hertz_t m_FrameRate;
    std::string m_CameraName;
    GeneratorType m_Generator;
    std::uniform_int_distribution<int> m_Distribution;
    Stopwatch m_FrameTimer;

    void fillRandom(uchar *start, uchar *end)
    {
        std::generate(start, end, [this](){ return static_cast<uchar>(m_Distribution(m_Generator)); });
    }

    // We (optionally) throttle the frame rate to a user-defined value
    bool frameReady()
    {
        using namespace units::time;
        using namespace std::literals;

        // No delay needed...
        if (std::isinf(m_FrameRate.value())) {
            return true;
        }

        const auto frameDelay = static_cast<Stopwatch::Duration>(1 / m_FrameRate);
        if (m_FrameTimer.elapsed() > frameDelay) {
            m_FrameTimer.start();
            return true;
        }

        return false;
    }
};
} // Video
} // BoBRobotics
