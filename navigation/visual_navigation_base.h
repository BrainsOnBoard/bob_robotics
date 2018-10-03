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
#include "../common/image_database.h"

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
        if (!tryTrain(imagePath, resizeImage)) {
            throw std::runtime_error("Path " + imagePath.str() + " does not exist");
        }
    }

    //! Train algorithm with specified route
    void trainRoute(const filesystem::path &routePath, bool resizeImages = false)
    {
        if (!routePath.exists()) {
            throw std::runtime_error("Path " + routePath.str() + " does not exist");
        }

        for (size_t i = 0;; i++) {
            const auto filename = routePath / getRouteDatabaseFilename(i);
            if (!tryTrain(filename, resizeImages)) {
                return;
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

    bool tryTrain(const filesystem::path &imagePath, bool resizeImage) noexcept
    {
        if (!imagePath.exists()) {
            return false;
        }

        // Load image
        cv::Mat image = cv::imread(imagePath.str(), cv::IMREAD_GRAYSCALE);
        BOB_ASSERT(image.type() == CV_8UC1);
        if (resizeImage) {
            cv::resize(image, image, m_UnwrapRes);
        } else {
            BOB_ASSERT(image.cols == m_UnwrapRes.width);
            BOB_ASSERT(image.rows == m_UnwrapRes.height);
        }

        // Add snapshot
        train(image);

        return true;
    }
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics