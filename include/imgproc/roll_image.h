#pragma once

// BoB robotics includes
#include "common/range.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <iterator>

namespace BoBRobotics {
namespace ImgProc {
template<class It1, class It2, class It3>
void
rollImage(const It1 begin, const It2 end, size_t stride, It3 out, int rotationRight)
{
    for (auto it = begin; it < end; std::advance(it, stride), std::advance(out, stride)) {
        std::rotate_copy(it, std::next(it, rotationRight), std::next(it, stride), out);
    }
}

/**!
 * \brief Rolls panoramic imageIn rotationRight pixels to the right, putting the
 *        result in imageOut.
 *
 * This function assumes that imageIn and imageOut are both continuous and that
 * their types match what is specified by T.
 */
template<class T>
void
rollImage(const cv::Mat &imageIn, cv::Mat &imageOut, int rotationRight)
{
    const auto begin = reinterpret_cast<T *>(imageIn.data);
    const auto end = &begin[imageIn.rows * imageIn.cols];
    const auto out = reinterpret_cast<T *>(imageOut.data);
    rollImage(begin, end, imageIn.cols, out, rotationRight);
}

template<class PixelType, class Func, class Container>
void
forEachRotation(const cv::Mat &image, Func func, const Container &columns)
{
    BOB_ASSERT(image.depth() == cv::DataType<PixelType>::depth);

#pragma omp parallel
    {
        static cv::Mat scratchImage;
#pragma omp threadprivate(scratchImage)
        /*
        * Allocate scratchImage lazily. This means it will always have the
        * correct size, even if the size of image changes between calls.
        */
        scratchImage.create(image.size(), image.type());

#pragma omp for
        for (auto column = columns.begin(); column < columns.end(); ++column) {
            rollImage<PixelType>(image, scratchImage, *column);
            func(scratchImage, std::distance(columns.begin(), column));
        }
    }
}

template<class PixelType, class Func>
void
forEachRotation(const cv::Mat &image, Func func)
{
    BOB_ASSERT(image.depth() == cv::DataType<PixelType>::depth);

#pragma omp parallel
    {
        static cv::Mat scratchImage;
#pragma omp threadprivate(scratchImage)
        /*
        * Allocate scratchImage lazily. This means it will always have the
        * correct size, even if the size of image changes between calls.
        */
        scratchImage.create(image.size(), image.type());

#pragma omp for
        for (size_t column = 0; column < image.cols; ++column) {
            rollImage<PixelType>(image, scratchImage, column);
            func(scratchImage, column);
        }
    }
}

} // ImgProc
} // BoBRobotics
