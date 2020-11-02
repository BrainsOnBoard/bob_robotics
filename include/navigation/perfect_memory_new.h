#pragma once

// BoB robotics includes
#include "algorithms.h"
#include "common/macros.h"
#include "common/range.h"
#include "differencers_new.h"
#include "ridf_processors_new.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
template<class T, class SizeType, class RIDFProcessor = BestMatchingSnapshot,
         class Differencer = AbsDiff>
class PerfectMemory
{
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
        std::transform(m_Snapshots.cbegin(), m_Snapshots.cend(), diffs, getDifference);
    }

    auto testAll(const T &image) const
    {
        std::vector<float> out;
        out.reserve(m_Snapshots.size());
        testAll(image, out.begin());

        return out;
    }

    const auto &getSnapshots() const
    {
        return m_Snapshots;
    }

    // template<class Container>
    // const auto &getImageDifferences(const cv::Mat &image, const Container &columns) const
    // {
    //     const auto &snapshots = this->getSnapshots();
    //     BOB_ASSERT(!snapshots.empty());

    //     // Preallocate snapshot difference vectors
    //     m_RotatedDifferences.resize(snapshots.size(), columns.size());

    //     calculateRIDFMulti(image, snapshots, m_RotatedDifferences,
    //                        Differencer{}, columns);

    //     return m_RotatedDifferences;
    // }

    const auto &getImageDifferences(const cv::Mat &image) const
    {
        const auto &snapshots = this->getSnapshots();
        BOB_ASSERT(!snapshots.empty());

        // Preallocate snapshot difference vectors
        m_RotatedDifferences.resize(snapshots.size(), m_Size.width);

        calculateRIDFMulti(image, snapshots, m_RotatedDifferences,
                           Differencer{});

        return m_RotatedDifferences;
    }

    // const auto &getImageDifferences(const cv::Mat &image) const
    // {
    //     return getImageDifferences(image, createRange(image.cols));
    // }

    const auto &getSnapshotBestColumns(const Eigen::MatrixXf &differences) const
    {
        const size_t numSnapshots = this->getSnapshots().size();
        m_SnapshotBestColumns.resize(numSnapshots);

#pragma omp parallel for
        for (size_t i = 0; i < numSnapshots; i++) {
            auto &cur = m_SnapshotBestColumns[i];
            cur.second = differences.row(i).minCoeff(&cur.first);
        }

        return m_SnapshotBestColumns;
    }

    const auto &getSnapshotBestColumns() const
    {
        return getSnapshotBestColumns(m_RotatedDifferences);
    }

    template<class... Ts>
    auto getHeadingAndSnapshotInfo(Ts &&... args) const
    {
        getImageDifferences(std::forward<Ts>(args)...);
        return RIDFProcessor{}(getSnapshotBestColumns(), m_Size.width);
    }

    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        return std::get<0>(getHeadingAndSnapshotInfo(std::forward<Ts>(args)...));
    }

private:
    mutable Eigen::MatrixXf m_RotatedDifferences;
    mutable std::vector<std::pair<size_t, float>> m_SnapshotBestColumns;
    std::vector<T> m_Snapshots;
    const SizeType m_Size;
}; // PerfectMemory
} // Navigation
} // BoBRobotics
