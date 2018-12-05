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

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Navigation {

namespace Internal {
template<typename T1, typename T2>
static auto
circularMean(const T1 &angles, const T2 &weights)
{
    units::dimensionless::scalar_t sumCos = 0.0, sumSin = 0.0;
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
        const auto bestSnapshot = static_cast<size_t>(std::distance(std::begin(minDifferences), bestPtr));

        // If column is > 180 deg, then subtract 360 deg
        int col = bestCols[bestSnapshot];
        if (col > (unwrapRes.width / 2)) {
            col -= unwrapRes.width;
        }

        // Convert to radians
        using namespace units::angle;
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
        using namespace units::angle;

        // Create vector of indices
        std::vector<size_t> idx(minDifferences.size());
        std::iota(std::begin(idx), std::end(idx), 0);

        // Build a heap (only partially sorts)
        auto comparator = [&minDifferences](const size_t i1, const size_t i2) {
            return minDifferences[i1] >= minDifferences[i2];
        };
        std::make_heap(std::begin(idx), std::end(idx), comparator);

        // Pop the best numComp indices off the heap into the output array
        std::array<size_t, numComp> snapshots;
        for(size_t &s : snapshots) {
            std::pop_heap(std::begin(idx), std::end(idx), comparator);
            s = idx.back();
            idx.pop_back();
        }

        // Convert best columns to headings
        auto colsToHeadings = [&bestCols, &unwrapRes](const size_t s) {
            return units::make_unit<turn_t>((double) bestCols[s] / (double) unwrapRes.width);
        };
        std::array<radian_t, numComp> headings;
        std::transform(snapshots.cbegin(), snapshots.cend(), headings.begin(),
                       colsToHeadings);

        // Normalise min differences to be between 0 and 1
        auto normaliseDiffs = [&minDifferences](const size_t i) {
            return minDifferences[i] / 255.0f;
        };
        std::array<float, numComp> minDifferencesOut;
        std::transform(snapshots.cbegin(), snapshots.cend(),
                       minDifferencesOut.begin(), normaliseDiffs);

        // Weights are 1 - min differences
        const auto diffsToWeights = [](const float f) {
            return 1.0f - f;
        };
        std::array<float, numComp> weights;
        std::transform(minDifferencesOut.cbegin(), minDifferencesOut.cend(),
                       weights.begin(), diffsToWeights);

        // Best angle is a weighted cirular mean of headings
        const radian_t bestAngle = Internal::circularMean(headings, weights);

        // Bundle result as tuple
        return std::make_tuple(bestAngle, std::move(snapshots), std::move(minDifferencesOut));
    }
};
} // Navigation
} // BoBRobotics
