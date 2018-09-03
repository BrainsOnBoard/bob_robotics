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
    VisualNavigationBase(const cv::Size unwrapRes, const unsigned int scanStep)
      : m_UnwrapRes(unwrapRes)
      , m_ScanStep(scanStep)
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
        assert(m_MaskImage.cols == m_UnwrapRes.width);
        assert(m_MaskImage.rows == m_UnwrapRes.height);
        assert(m_MaskImage.type() == CV_8UC1);
    }

    //! Number of pixels for each scanning "step" when doing RIDF
    inline unsigned int getScanStep() const
    {
        return m_ScanStep;
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
    const unsigned int m_ScanStep;
    cv::Mat m_MaskImage;

    bool tryTrain(const filesystem::path &imagePath, bool resizeImage) noexcept
    {
        if (!imagePath.exists()) {
            return false;
        }

        // Load image
        cv::Mat image = cv::imread(imagePath.str(), cv::IMREAD_GRAYSCALE);
        assert(image.type() == CV_8UC1);
        if (resizeImage) {
            cv::resize(image, image, m_UnwrapRes);
        } else {
            assert(image.cols == m_UnwrapRes.width);
            assert(image.rows == m_UnwrapRes.height);
        }

        // Add snapshot
        train(image);

        return true;
    }
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics