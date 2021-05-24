#pragma once

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstddef>

namespace BoBRobotics
{
namespace ImgProc
{
//! A class providing extra checks and type safety for image masks
class Mask
{
public:
    //! Create an empty mask (i.e. treat all pixels as unmasked)
    Mask() = default;

    //! Construct a mask based on a cv::Mat where zero pixels are treated as masked
    explicit Mask(cv::Mat mask, const cv::Size &resizeTo = {});

    //! Load a mask from an image file
    explicit Mask(const filesystem::path &imagePath, const cv::Size &resizeTo = {});

    //! Zero out the masked pixels in `in` and copy the result to `out`
    void apply(const cv::Mat &in, cv::Mat &out) const;

    //! Combine this mask with another, saving to outputMask
    void combine(const Mask &otherMask, Mask &outputMask) const;

    size_t countUnmaskedPixels(const cv::Size &size) const;
    bool empty() const;
    const cv::Mat &get() const;
    bool isValid(const cv::Size &imageSize) const;

    /*!
     * \brief Roll the mask the specified number of pixels to the left
     *
     * Internally this just uses ImgProc::roll, but this method is provided for
     * type safety.
     */
    void roll(Mask &out, size_t pixelsLeft) const;

    void set(cv::Mat mask, const cv::Size &resizeTo = {});
    void set(const filesystem::path &imagePath, const cv::Size &resizeTo = {});

private:
    cv::Mat m_Mask;
};
} // ImgProc
} // BoBRobotics
