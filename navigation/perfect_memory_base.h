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
#include "visual_navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;
using namespace units::angle;
using namespace units::dimensionless;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
//! An abstract class which is the base for PerfectMemory and PerfectMemoryHOG
template<typename RIDFProcessor>
class PerfectMemoryBase
  : public VisualNavigationBase
{
public:
    PerfectMemoryBase(const cv::Size unwrapRes, const unsigned int scanStep = 1)
      : VisualNavigationBase(unwrapRes, scanStep)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Return the number of snapshots that have been read into memory
    virtual size_t getNumSnapshots() const = 0;

    //! Return a specific snapshot
    virtual const cv::Mat &getSnapshot(size_t index) const = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Add snapshot
        addSnapshot(image);
    }

    //! Get differences between image and stored snapshots
    std::vector<std::vector<float>> getImageDifferences(const cv::Mat &image) const
    {
        assert(getNumSnapshots() > 0);

        // Create vector to store RIDF values
        std::vector<std::vector<float>> differences(getNumSnapshots());
        for (auto &d : differences) {
            d.reserve(image.cols / getScanStep());
        }

        auto saveDifference = [this, &differences] (auto rollImage, auto maskImage) {
            // Loop through snapshots
            for (size_t s = 0; s < getNumSnapshots(); s++) {
                // Calculate difference
                differences[s].emplace_back(calcSnapshotDifference(rollImage, maskImage, s));
            }
        };
        rollImageTransform(image, saveDifference);

        return differences;
    }

    //! Get an estimate for heading based on comparing image with stored snapshots
    auto getHeading(const cv::Mat &image) const
    {
        std::vector<std::vector<float>> differences = getImageDifferences(image);
        const size_t numSnapshots = getNumSnapshots();

        // Now get the minimum for each snapshot and the column this corresponds to
        std::vector<int> bestColumns(numSnapshots);
        std::vector<float> minDifferences(numSnapshots);
        for (size_t i = 0; i < numSnapshots; i++) {
            const auto elem = std::min_element(std::cbegin(differences[i]), std::cend(differences[i]));
            bestColumns[i] = std::distance(std::cbegin(differences[i]), elem);
            minDifferences[i] = *elem;
        }

        // Return result
        return std::tuple_cat(RIDFProcessor()(getUnwrapResolution(), bestColumns, minDifferences),
                              std::make_tuple(std::move(differences)));
    }

protected:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) = 0;

    //! Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const = 0;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics