#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
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
    VisualNavigationBase(const cv::Size unwrapRes)
      : m_UnwrapRes(unwrapRes)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Train with image at specified path
    void train(const filesystem::path &imagePath, bool resizeImage = false)
    {
        if (!imagePath.exists()) {
            throw std::runtime_error("Path " + imagePath.str() + " does not exist");
        }

        // Load image
        cv::Mat image = cv::imread(imagePath.str(), cv::IMREAD_GRAYSCALE);
        train(image, resizeImage);
    }

    void train(const cv::Mat &image, bool resizeImage)
    {
        assert(image.type() == CV_8UC1);
        if (resizeImage) {
            cv::Mat tmp;
            cv::resize(image, tmp, m_UnwrapRes);
            train(tmp);
        } else {
            assert(image.cols == m_UnwrapRes.width);
            assert(image.rows == m_UnwrapRes.height);
            train(image);
        }
    }

    //! Train algorithm with specified route
    void trainRoute(const ImageDatabase &imdb, bool resizeImages = false)
    {
        for (const auto &e : imdb) {
            train(e.loadGreyscale(), resizeImages);
        }
    }

    //! Set mask image (e.g. for masking part of robot)
    void setMaskImage(const std::string &path)
    {
        m_MaskImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
        assert(m_MaskImage.cols == m_UnwrapRes.width);
        assert(m_MaskImage.rows == m_UnwrapRes.height);
        assert(m_MaskImage.type() == CV_8UC1);
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