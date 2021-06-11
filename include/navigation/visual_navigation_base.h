#pragma once

// BoB robotics includes
#include "image_database.h"
#include "imgproc/mask.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <string>
#include <vector>

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
    virtual void train(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) = 0;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image, const ImgProc::Mask &mask = ImgProc::Mask{}) const = 0;

    //! Clears the training from memory
    virtual void clearMemory() = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    //! Train algorithm with specified route
    void trainRoute(const std::vector<cv::Mat> &images);
    void trainRoute(const ImageDatabase &imdb, bool resizeImages = false);

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const;

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    const cv::Size m_UnwrapRes;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics
