#pragma once

// BoB robotics includes
#include "common/circstat.h"

// Third-party includes
#include "third_party/units.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <limits>
#include <numeric>
#include <tuple>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//! Winner-take all: derive heading using only the best-matching snapshot
struct BestMatchingSnapshot
{
    auto operator()(const std::vector<std::pair<size_t, float>> &bestCols,
                    size_t imWidth)
    {
        // Get index corresponding to best-matching snapshot
        const auto best = std::min_element(std::begin(bestCols), std::end(bestCols));

        // Convert to radians
        using namespace units::angle;
        // const radian_t heading = units::make_unit<turn_t>((double) col / (double) unwrapRes.width);
        degree_t heading = turn_t{ (double) best->first / (double) imWidth };
        heading = normaliseAngle180(heading);

        // Normalise to be between 0 and 1
        const float difference = best->second / 255.0f;

        // Bundle result as tuple
        return std::make_tuple(heading, (size_t) std::distance(bestCols.begin(), best), difference);
    }
};

//! Dynamic weighting: use weighted average of $n$ best-matching snapshots' headings
template<size_t numComp>
struct WeightSnapshotsDynamic
{
    template<typename Rotater>
    auto operator()(std::vector<std::pair<size_t, float>> &bestCols,
                    const Rotater &rotater)
    {
        using namespace units::angle;

        // Create vector of indices
        std::vector<size_t> idx(bestCols.size());
        std::iota(std::begin(idx), std::end(idx), 0);

        // Build a heap (only partially sorts)
        auto comparator = [&bestCols](const size_t i1, const size_t i2) {
            return bestCols[i1].second >= bestCols[i2].second;
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
        auto colsToHeadings = [&bestCols, &rotater](const size_t s) {
            return rotater.columnToHeading(bestCols[s].first);
        };
        std::array<radian_t, numComp> headings;
        std::transform(snapshots.cbegin(), snapshots.cend(), headings.begin(),
                       colsToHeadings);

        // Normalise min differences to be between 0 and 1
        auto normaliseDiffs = [&bestCols](const size_t i) {
            return bestCols[i].second / 255.0f;
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
        const radian_t bestAngle = circularMean(headings, weights);

        // Bundle result as tuple
        return std::make_tuple(bestAngle, std::move(snapshots), std::move(minDifferencesOut));
    }
};
} // Navigation
} // BoBRobotics
