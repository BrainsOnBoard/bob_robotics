/*
 * This header contains small, functor-type classes for calculating the
 * differences between images, used by PerfectMemory.
 */

#pragma once

// Standard C includes
#include <cmath>
#include <cstdlib>

// Standard C++ includes
#include <algorithm>
#include <type_traits>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace Navigation {
namespace Internal {
    inline uint8_t *begin(const cv::Mat &image)
    {
        return image.data;
    }

    inline uint8_t *end(const cv::Mat &image)
    {
        return &image.data[image.cols * image.rows];
    }

    template<typename T>
    inline auto begin(const T &input)
    {
        return std::begin(input);
    }

    template<typename T>
    inline auto end(const T &input)
    {
        return std::end(input);
    }
}

//------------------------------------------------------------------------
// BoBRobotics::Navigation::AbsDiff
//------------------------------------------------------------------------
/*!
 * \brief For calculating the mean absolute difference between images
 *
 * Can be passed to PerfectMemory as a template parameter.
 */
class AbsDiff
{
public:
    AbsDiff(const int) {}

    template<typename InputArray1, typename InputArray2, typename OutputArray>
    inline auto operator()(const InputArray1 &src1, const InputArray2 &src2,
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
class RMSDiff
{
    public:
    RMSDiff(const size_t imSize) : m_Differences(imSize)
    {}

    template<typename InputArray1, typename InputArray2, typename OutputArray>
    inline std::vector<float>::iterator operator()(const InputArray1 &src1,
                                                   const InputArray2 &src2,
                                                   OutputArray &dst)
    {
        cv::absdiff(src1, src2, dst);
        const auto sqDiff = [](const auto val)
        {
            const auto d = static_cast<float>(val);
            return d * d;
        };
        std::transform(Internal::begin(dst), Internal::end(dst), std::begin(m_Differences), sqDiff);
        return m_Differences.begin();
    }

    static inline float mean(const float sum, const float n)
    {
        return sqrt(sum / n);
    }

    private:
    std::vector<float> m_Differences;
};
} // Navigation
} // BoBRobotics
