#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Navigation {

template<class It1, class It2, class It3>
void __rollImage(const It1 begin, const It2 end, size_t stride, It3 out, size_t rotationRight)
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
void rollImage(const cv::Mat &imageIn, cv::Mat &imageOut, size_t rotationRight)
{
    const auto begin = reinterpret_cast<T *>(imageIn.data);
    const auto end = &begin[imageIn.rows * imageIn.cols];
    const auto out = reinterpret_cast<T *>(imageOut.data);
    __rollImage(begin, end, imageIn.cols, out, rotationRight);
}
} // Navigation
} // BoBRobotics
