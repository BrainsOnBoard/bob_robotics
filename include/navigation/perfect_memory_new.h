#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "algorithms.h"
#include "differencers_new.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <algorithm>
#include <iterator>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
template<class T, class SizeType, class Differencer = AbsDiff>
class PerfectMemory {
public:
    PerfectMemory(const SizeType &size)
      : m_Size(size)
    {}

    void train(const T &image)
    {
        BOB_ASSERT(image.size() == m_Size);
        m_Snapshots.emplace_back(image);
    }

    float test(const T &image) const
    {
        const auto diffs = testAll(image);
        return *std::min_element(diffs.cbegin(), diffs.cend());
    }

    template<class Iter>
    void testAll(const T &image, Iter diffs)
    {
        const auto getDifference = [&image](const auto &snap) {
            return Differencer::calculate(image, snap);
        };
        std::transform(m_Snapshots.cbegin(), m_Snapshots.cend(), diffs,
                       getDifference);
    }

    auto testAll(const T &image) const
    {
        std::vector<float> out;
        out.reserve(m_Snapshots.size());
        testAll(image, out.begin());

        return out;
    }

    const auto &getSnapshots() const { return m_Snapshots; }

private:
    std::vector<T> m_Snapshots;
    const SizeType m_Size;
}; // PerfectMemory

template<class BaseClass>
class PerfectMemoryRotater
 : BaseClass
{
public:
    const auto &getImageDifferences(const cv::Mat &image, size_t columnStep,
                                    size_t columnBegin, size_t columnEnd) const
    {
        const auto &snapshots = this->getSnapshots();
        BOB_ASSERT(!snapshots.empty());

        // Preallocate snapshot difference vectors
        size_t rotations = (columnEnd - columnBegin) / columnStep;
        m_RotatedDifferences.resize(snapshots.size(), rotations);

        calculateRIDFMulti(image, snapshots, m_RotatedDifferences);

        return m_RotatedDifferences;
    }

private:
    mutable Eigen::MatrixXf m_RotatedDifferences;
    mutable std::vector<size_t> m_BestColumns;
    mutable std::vector<float> m_MinimumDifferences;
}; // PerfectMemory
} // Navigation
} // BoBRobotics
