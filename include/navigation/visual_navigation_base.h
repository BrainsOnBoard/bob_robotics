#pragma once

// BoB robotics includes
#include "image_database.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <string>

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::VisualNavigationBase
//------------------------------------------------------------------------
//! The base class for visual navigation algorithms (currently just variations on perfect memory)
class VisualNavigationBase
{
public:
    VisualNavigationBase(const cv::Size &unwrapRes);

    virtual ~VisualNavigationBase();

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
    void trainRoute(const ImageDatabase &imdb, bool resizeImages = false);

    //! Set mask image (e.g. for masking part of robot)
    void setMaskImage(const std::string &path);

    //! Return mask image
    const cv::Mat &getMaskImage() const;

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const;

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    const cv::Size m_UnwrapRes;
    cv::Mat m_MaskImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics
