/*
 * This header contains small, functor-type classes for calculating the
 * differences between images, used by PerfectMemory.
 */

#pragma once

// BoB robotics includes
#include "common/macros.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <iterator>
#include <type_traits>
#include <numeric>
#include <utility>
#include <vector>

// Standard C includes
#include <cmath>
#include <cstdlib>

namespace BoBRobotics {
namespace Navigation {
namespace Internal {
inline uint8_t *
begin(const cv::Mat &image)
{
    return image.data;
}

inline uint8_t *
end(const cv::Mat &image)
{
    return &image.data[image.cols * image.rows];
}

template<typename T>
inline auto
begin(const T &input)
{
    return std::begin(input);
}

template<typename T>
inline auto
end(const T &input)
{
    return std::end(input);
}
}

template<class Iter>
std::pair<float, size_t>
maskedSum(Iter it, size_t count, const cv::Mat &mask1, const cv::Mat &mask2)
{
    BOB_ASSERT(mask1.size() == mask2.size());

    // If there's no mask
    if (mask1.empty()) {
        float sum = std::accumulate(it, it + count, 0.0f);
        return { sum, count };
    }

    uint8_t *maskPtr1 = mask1.data;
    uint8_t *maskPtr2 = mask2.data;

    float sum = 0.0f;
    size_t numUnmaskedPixels = 0;
    while (maskPtr1 < mask1.dataend) {
        // If this pixel is masked by neither of the masks
        if (*maskPtr1 && *maskPtr2) {
            // Accumulate sum of differences
            sum += (float) *it;

            // Increment unmasked pixels count
            numUnmaskedPixels++;
        }
        ++maskPtr1;
        ++maskPtr2;
        ++it;
    }

    return { sum, numUnmaskedPixels };
}

//------------------------------------------------------------------------
// BoBRobotics::Navigation::AbsDiff
//------------------------------------------------------------------------
/*!
 * \brief For calculating the mean absolute difference between images
 *
 * Can be passed to PerfectMemory as a template parameter.
 */
struct AbsDiff
{
    template<typename InputArray1, typename InputArray2, typename OutputArray>
    static auto calculate(const InputArray1 &src1, const InputArray2 &src2,
                          OutputArray &dst)
    {
        // Calculate absdiff
        cv::absdiff(src1, src2, dst);

        // Return copy of output iterator
        return Internal::begin(dst);
    }

    static inline float mean(const float sum, const float n)
    {
        return sum / n;
    }
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::RMSDiff
//------------------------------------------------------------------------
/*!
 * \brief For calculating the root mean square difference between images
 *
 * Can be passed to PerfectMemory as a template parameter.
 */
struct RMSDiff
{
    template<typename InputArray1, typename InputArray2, typename OutputArray>
    static auto calculate(const InputArray1 &src1,
                          const InputArray2 &src2, OutputArray &dst)
    {
        static std::vector<float> diffs;
        #pragma omp threadprivate(diffs)
        diffs.clear();

        cv::absdiff(src1, src2, dst);

        const auto sqDiff = [](const auto val) {
            const auto d = static_cast<float>(val);
            return d * d;
        };
        std::transform(Internal::begin(dst), Internal::end(dst), std::back_inserter(diffs), sqDiff);
        return diffs.begin();
    }

    static inline float mean(const float sum, const float n)
    {
        return sqrt(sum / n);
    }
};
} // Navigation
} // BoBRobotics
