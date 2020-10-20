#pragma once

// Standard C++ includes
#include <numeric>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
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
    template<class Array>
    float operator()(const Array &src1, const Array &src2) const
    {
        BOB_ASSERT(src1.depth() == CV_32F);

        static cv::Mat dst;
#pragma omp threadprivate(dst)
        dst.create(src1.size(), src1.type());

        cv::absdiff(src1, src2, dst);
        const auto dstPtr = reinterpret_cast<float *>(dst.data);
        std::vector<float> tmp;
        std::copy(&dstPtr[0], &dstPtr[dst.total()], std::back_inserter(tmp));
        float sum = std::accumulate(&dstPtr[0], &dstPtr[dst.total()], 0.f);

        // cv::imshow("difference", dst);
        // cv::waitKey(0);

        return sum / (float) dst.total();
    }
}; // AbsDiff
} // Navigation
} // BoBRobotics
