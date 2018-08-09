#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/image_database.h"

// Third-party includes
#include "../third_party/units.h"

// Local includes
#include "navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;
using namespace units::angle;
using namespace units::dimensionless;

template<typename T1, size_t N, typename T2>
T1 circularMean(const std::array<T1, N> &angles, const T2 &weights)
{
    scalar_t sumCos = 0.0;
    scalar_t sumSin = 0.0;
    for (size_t i = 0; i < N; i++) {
        sumCos += weights[i] * units::math::cos(angles[i]);
        sumSin += weights[i] * units::math::sin(angles[i]);
    }

    return units::math::atan2(sumSin / N, sumCos / N);
}

class BestMatchingSnapshot
{
public:
    inline void operator()(float difference, int col, size_t snapshot)
    {
        if (difference < m_MinDifference) {
            m_MinDifference = difference;
            m_BestCol = col;
            m_BestSnapshot = snapshot;
        }
    }

    inline auto result(const cv::Size &unwrapRes)
    {
        // If best column is more than 180 degrees away, flip
        if (m_BestCol > (unwrapRes.width / 2)) {
            m_BestCol -= unwrapRes.width;
        }

        // Convert column into angle
        const radian_t bestAngle = units::make_unit<turn_t>((double) m_BestCol / (double) unwrapRes.width);

        // Bundle up result as a tuple
        return std::make_tuple(bestAngle, m_BestSnapshot, m_MinDifference);
    }

    inline size_t getBestSnapshot() const
    {
        return m_BestSnapshot;
    }

private:
    float m_MinDifference = std::numeric_limits<float>::infinity();
    int m_BestCol;
    size_t m_BestSnapshot;
};

template<size_t numSnapshots>
class WeightNSnapshots
{
public:
    WeightNSnapshots()
    {
        m_MinDifferences.fill(std::numeric_limits<float>::infinity());
    }

    inline void operator()(float difference, int col, size_t snapshot)
    {
        size_t pos = numSnapshots;
        for (; pos >= 0 && difference < m_MinDifferences[pos]; pos--);
        if (pos < numSnapshots) {
            // Shift values in the array down one rank
            shiftfrom(m_MinDifferences, pos);
            shiftfrom(m_BestCols, pos);
            shiftfrom(m_BestSnapshots, pos);

            // Put new values in the right place
            m_MinDifferences[pos] = difference;
            m_BestCols[pos] = col;
            m_BestSnapshots[pos] = snapshot;
        }
    }

    inline auto result(const cv::Size &unwrapRes)
    {
        // Normalise weights
        const float sumWeights = std::accumulate(m_MinDifferences.begin(), m_MinDifferences.end(), 0.0f);
        std::array<float, numSnapshots> weights;
        std::transform(m_MinDifferences.begin(), m_MinDifferences.end(), weights.begin(), [sumWeights](float val) {
            return val / sumWeights;
        });

        // Turn best column values into headings
        std::array<radian_t, numSnapshots> headings;
        std::transform(m_BestCols.begin(), m_BestCols.end(), headings.begin(), [unwrapRes](int col) {
            return units::make_unit<turn_t>((double) col / (double) unwrapRes.width);
        });

        // Best angle is a weighted cirular mean of headings
        const radian_t bestAngle = circularMean(headings, weights);

        return std::make_tuple(bestAngle, std::move(m_BestSnapshots), std::move(m_MinDifferences));
    }

    inline size_t getBestSnapshot() const
    {
        return m_BestSnapshots[0];
    }

private:
    std::array<float, numSnapshots> m_MinDifferences;
    std::array<int, numSnapshots> m_BestCols;
    std::array<size_t, numSnapshots> m_BestSnapshots;

    template<typename T>
    static inline void shiftfrom(T array, size_t pos)
    {
        std::copy_backward(&array[pos], array.end() - 1, array.end());
    }
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
template<typename RIDFProcessor = BestMatchingSnapshot>
class PerfectMemoryBase
  : public NavigationBase
{
private:
    class RIDFValueLogger
      : public RIDFProcessor
    {
    public:
        inline void operator()(float difference, int col, size_t snapshot)
        {
            if (snapshot != m_CurrentSnapshot) {
                snapshotUpdate();
                m_CurrentSnapshot = snapshot;
                m_CurrentDifferences.clear();
            }

            RIDFProcessor::operator()(difference, col, snapshot);
            m_CurrentDifferences.push_back(difference);
        }

        inline auto result(const cv::Size &unwrapRes)
        {
            snapshotUpdate();
            const auto res = RIDFProcessor::result(unwrapRes);
            return std::make_tuple(std::get<0>(res), std::get<1>(res), std::get<2>(res), std::move(m_BestDifferences));
        }

    private:
        std::vector<float> m_BestDifferences, m_CurrentDifferences;
        size_t m_CurrentSnapshot = 0;

        void snapshotUpdate()
        {
            if (RIDFProcessor::getBestSnapshot() == m_CurrentSnapshot) {
                m_BestDifferences.resize(m_CurrentDifferences.size());
                std::copy(m_CurrentDifferences.begin(), m_CurrentDifferences.end(), m_BestDifferences.begin());
            }
        }
    };

    template<typename T>
    auto runRIDF(const cv::Mat &image, T &&processor) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Clone mask and image so they can be rolled inplace
        getMaskImage().copyTo(m_ScratchMaskImage);
        image.copyTo(m_ScratchRollImage);

        // Scan across image columns
        const size_t numSnapshots = getNumSnapshots();
        const size_t scanStep = getScanStep();
        for (int i = 0; i < m_ScratchRollImage.cols; i += scanStep) {
            // Loop through snapshots
            for (size_t s = 0; s < numSnapshots; s++) {
                // Calculate difference
                const float difference = calcSnapshotDifference(m_ScratchRollImage, m_ScratchMaskImage, s);
                processor(difference, i, s);
            }

            // Roll image and corresponding mask left by scanStep
            rollImage(m_ScratchRollImage);
            if (!m_ScratchMaskImage.empty()) {
                rollImage(m_ScratchMaskImage);
            }
        }

        // Return result
        return processor.result(unwrapRes);
    }

public:
    PerfectMemoryBase(const cv::Size unwrapRes, const unsigned int scanStep = 1, const filesystem::path outputPath = "snapshots")
      : NavigationBase(unwrapRes, scanStep, outputPath)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const = 0;
    virtual const cv::Mat &getSnapshot(size_t index) const = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image, bool saveImage = false) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Add snapshot
        const size_t index = addSnapshot(image);

        // Write image to disk, if desired
        if (saveImage) {
            saveSnapshot(index, image);
        }
    }

    auto getHeading(const cv::Mat &image) const
    {
        return runRIDF(image, RIDFProcessor());
    }

    auto getRIDF(const cv::Mat &image) const
    {
        return runRIDF(image, RIDFValueLogger());
    }

protected:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    // Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) = 0;

    // Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const = 0;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    filesystem::path getSnapshotPath(size_t index) const
    {
        return getOutputPath() / getRouteDatabaseFilename(index);
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable cv::Mat m_ScratchMaskImage;
    mutable cv::Mat m_ScratchRollImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics