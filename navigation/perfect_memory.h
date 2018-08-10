#pragma once

// Standard C++ includes
#include <numeric>
#include <vector>

// Standard C includes
#include <cassert>
#include <cstdlib>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/path.h"

// Local includes
#include "perfect_memory_base.h"

namespace BoBRobotics {
namespace Navigation {

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
        return std::make_tuple(bestAngle, m_BestSnapshot, m_MinDifference / 255.0f);
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
        for (; pos > 0 && difference < m_MinDifferences[pos - 1]; pos--)
            ;
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
        // Normalise min differences to be between 0 and 1
        std::transform(m_MinDifferences.begin(), m_MinDifferences.end(), m_MinDifferences.begin(), [](float val) {
            return val / 255.0f;
        });

        // Weights are 1 minus min difference, normalised to sum to 1
        std::array<float, numSnapshots> weights;
        float sumWeights = 0.0f;
        for (size_t i = 0; i < numSnapshots; i++) {
            weights[i] = 1.0f - m_MinDifferences[i];
            sumWeights += weights[i];
        }
        std::transform(m_MinDifferences.begin(), m_MinDifferences.end(), weights.begin(), [sumWeights](float val) {
            return val / sumWeights;
        });

        // Turn best column values into headings
        std::array<radian_t, numSnapshots> headings;
        std::transform(m_BestCols.begin(), m_BestCols.end(), headings.begin(), [unwrapRes](int col) {
            // If best column is more than 180 degrees away, flip
            if (col > (unwrapRes.width / 2)) {
                col -= unwrapRes.width;
            }
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

    template<typename T1, size_t N, typename T2>
    static T1 circularMean(const std::array<T1, N> &angles, const T2 &weights)
    {
        scalar_t sumCos = 0.0;
        scalar_t sumSin = 0.0;
        for (size_t i = 0; i < N; i++) {
            sumCos += weights[i] * units::math::cos(angles[i]);
            sumSin += weights[i] * units::math::sin(angles[i]);
        }

        return units::math::atan2(sumSin / N, sumCos / N);
    }
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemory
//------------------------------------------------------------------------
template<typename RIDFProcessor = BestMatchingSnapshot>
class PerfectMemory
  : public PerfectMemoryBase<RIDFProcessor>
{
public:
    PerfectMemory(const cv::Size unwrapRes, const unsigned int scanStep = 1, const filesystem::path outputPath = "snapshots")
      : PerfectMemoryBase<RIDFProcessor>(unwrapRes, scanStep, outputPath)
      , m_DiffScratchImage(unwrapRes, CV_8UC1)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const override
    {
        return m_Snapshots.size();
    }
    virtual const cv::Mat &getSnapshot(size_t index) const override
    {
        return m_Snapshots[index];
    }

protected:
    //! Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) override
    {
        m_Snapshots.emplace_back();
        image.copyTo(m_Snapshots.back());

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    //! Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const override
    {
        // Calculate absolute difference between image and stored image
        cv::absdiff(m_Snapshots[snapshot], image, m_DiffScratchImage);

        // Get raw access to image difference values
        const uint8_t *rawDiff = reinterpret_cast<const uint8_t *>(m_DiffScratchImage.data);

        // If there's no mask
        if (imageMask.empty()) {
            const int len = m_DiffScratchImage.cols * m_DiffScratchImage.rows;
            float sumDifference = std::accumulate(rawDiff, rawDiff + len, 0.0f);

            // Return mean
            return sumDifference / (float) len;
        }
        // Otherwise
        else {
            // Get raw access to rotated mask associated with image and non-rotated mask associated with snapshot
            const uint8_t *rawImageMask = reinterpret_cast<const uint8_t *>(imageMask.data);
            const uint8_t *rawSnapshotMask = reinterpret_cast<const uint8_t *>(this->getMaskImage().data);

            // Loop through pixels
            float sumDifference = 0.0f;
            unsigned int numUnmaskedPixels = 0;
            for (int i = 0; i < (m_DiffScratchImage.cols * m_DiffScratchImage.rows); i++) {
                // If this pixel is masked by neither of the masks
                if (rawImageMask[i] != 0 && rawSnapshotMask[i] != 0) {
                    // Accumulate sum of differences
                    sumDifference += (float) rawDiff[i];

                    // Increment unmasked pixels count
                    numUnmaskedPixels++;
                }
            }

            // Return mean
            return sumDifference / (float) numUnmaskedPixels;
        }
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<cv::Mat> m_Snapshots;
    mutable cv::Mat m_DiffScratchImage;
}; // PerfectMemory
} // Navigation
} // BoBRobotics