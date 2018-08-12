#pragma once

// Standard C++ includes
#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics
#include "common/rtransform.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;

namespace Internal {
template<typename T1, typename T2>
static radian_t
circularMean(const T1 &angles, const T2 &weights)
{
    scalar_t sumCos = 0.0;
    scalar_t sumSin = 0.0;
    for (size_t i = 0; i < angles.size(); i++) {
        sumCos += weights[i] * units::math::cos(angles[i]);
        sumSin += weights[i] * units::math::sin(angles[i]);
    }

    return units::math::atan2(sumSin / angles.size(), sumCos / angles.size());
}
}

//! Winner-take all: derive heading using only the best-matching snapshot
struct BestMatchingSnapshot
{
    auto operator()(const cv::Size &unwrapRes,
                    std::vector<int> &bestCols,
                    std::vector<float> &minDifferences)
    {
        // Get index corresponding to best-matching snapshot
        const auto bestPtr = std::min_element(std::begin(minDifferences), std::end(minDifferences));
        const size_t bestSnapshot = std::distance(std::begin(minDifferences), bestPtr);

        // If column is > 180 deg, then subtract 360 deg
        int col = bestCols[bestSnapshot];
        if (col > (unwrapRes.width / 2)) {
            col -= unwrapRes.width;
        }

        // Convert to radians
        const radian_t heading = units::make_unit<turn_t>((double) col / (double) unwrapRes.width);

        // Normalise to be between 0 and 1
        const float difference = minDifferences[bestSnapshot] / 255.0f;

        // Bundle result as tuple
        return std::make_tuple(heading, bestSnapshot, difference);
    }
};

//! Dynamic weighting: use weighted average of $n$ best-matching snapshots' headings
template<size_t numComp>
struct WeightSnapshotsDynamic
{
    auto operator()(const cv::Size &unwrapRes,
                    std::vector<int> &bestCols,
                    std::vector<float> &minDifferences)
    {
        // Create vector of indices
        std::vector<size_t> idx(minDifferences.size());
        std::iota(std::begin(idx), std::end(idx), 0);

        // Sort indices by minimum difference values
        std::sort(std::begin(idx), std::end(idx), [&minDifferences](const size_t i1, const size_t i2) {
            return minDifferences[i1] < minDifferences[i2];
        });

        // Copy the best numComp indices to output array
        std::array<size_t, numComp> snapshots;
        std::copy_n(std::begin(idx), numComp, std::begin(snapshots));

        // Convert best columns to headings
        std::array<radian_t, numComp> headings;
        rtransform(snapshots, headings, [&bestCols, &unwrapRes](const size_t s) {
            return units::make_unit<turn_t>((double) bestCols[s] / (double) unwrapRes.width);
        });

        // Normalise min differences to be between 0 and 1
        std::array<float, numComp> minDifferencesOut;
        rtransform(snapshots, minDifferencesOut, [&minDifferences](const size_t i) {
            return minDifferences[i] / 255.0f;
        });

        // Weights are 1 - min differences
        std::array<float, numComp> weights;
        rtransform(minDifferencesOut, weights, [](const float f) {
            return 1.0f - f;
        });

        // Best angle is a weighted cirular mean of headings
        const radian_t bestAngle = 0_rad; // = Internal::circularMean(headings, weights);

        // Bundle result as tuple
        return std::make_tuple(bestAngle, std::move(snapshots), std::move(minDifferencesOut));
    }
};
} // Navigation
} // BoBRobotics