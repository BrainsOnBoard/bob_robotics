#pragma once

// Standard C includes
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/assert.h"
#include "image_database.h"

// Third-party includes
#include "../third_party/path.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::VisualNavigationBase
//------------------------------------------------------------------------
//! The base class for visual navigation algorithms (currently just variations on perfect memory)
class VisualNavigationBase
{
public:
    VisualNavigationBase(const cv::Size &unwrapRes)
      : m_UnwrapRes(unwrapRes)
    {}

    virtual ~VisualNavigationBase() {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) = 0;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) const = 0;

    //! Clears the training from memory
    virtual void clearMemory() = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Train algorithm with specified route
    void trainRoute(const ImageDatabase &imdb, bool resizeImages = false)
    {
        cv::Mat image;
        if (resizeImages) {
            cv::Mat imageResized;
            for (const auto &e : imdb) {
                image = e.loadGreyscale();
                BOB_ASSERT(image.type() == CV_8UC1);
                cv::resize(image, imageResized, m_UnwrapRes);
                train(imageResized);
            }
        } else {
            for (const auto &e : imdb) {
                image = e.loadGreyscale();
                BOB_ASSERT(image.type() == CV_8UC1);
                BOB_ASSERT(image.cols == m_UnwrapRes.width);
                BOB_ASSERT(image.rows == m_UnwrapRes.height);
                train(image);
            }
        }
    }

    //! Set mask image (e.g. for masking part of robot)
    void setMaskImage(const std::string &path)
    {
        m_MaskImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
        BOB_ASSERT(m_MaskImage.cols == m_UnwrapRes.width);
        BOB_ASSERT(m_MaskImage.rows == m_UnwrapRes.height);
        BOB_ASSERT(m_MaskImage.type() == CV_8UC1);
    }

    //! Return mask image
    const cv::Mat &getMaskImage() const
    {
        return m_MaskImage;
    }

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const
    {
        return m_UnwrapRes;
    }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    const cv::Size m_UnwrapRes;
    cv::Mat m_MaskImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics
