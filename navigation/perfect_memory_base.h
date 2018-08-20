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
using namespace units::angle;
using namespace units::dimensionless;
using namespace units::literals;

class InSilicoRotater
{
    public:
    InSilicoRotater(unsigned int scanStep, const cv::Mat &image)
      : m_CurrentRotation(0)
      , m_ScanStep(scanStep)
    {
        image.copyTo(m_ScratchImage);
    }

    cv::Mat &getImage()
    {
        return m_ScratchImage;
    }

    bool next()
    {
        int next = m_CurrentRotation + m_ScanStep;
        if (next >= m_ScratchImage.cols) {
            return false;
        }

        // Loop through rows
        for (int y = 0; y < m_ScratchImage.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = m_ScratchImage.ptr(y);

            // Rotate row to left by m_ScanStep pixels
            std::rotate(rowPtr, rowPtr + m_ScanStep, rowPtr + m_ScratchImage.cols);
        }
        m_CurrentRotation = next;

        return true;
    }

    size_t count() const
    {
        return m_CurrentRotation / m_ScanStep;
    }

    size_t max() const
    {
        return m_ScratchImage.cols / m_ScanStep;
    }

    private:
    int m_CurrentRotation;
    const unsigned int m_ScanStep;
    cv::Mat m_ScratchImage;
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
//! An abstract class which is the base for PerfectMemory and PerfectMemoryHOG
template<typename RIDFProcessor, typename Rotater = InSilicoRotater>
class PerfectMemoryBase
  : public VisualNavigationBase
{
public:
    PerfectMemoryBase(const cv::Size unwrapRes, const unsigned int scanStep = 1,
                      const filesystem::path outputPath = "snapshots")
      : VisualNavigationBase(unwrapRes, scanStep, outputPath)
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

    //! Get differences between image and stored snapshots
    std::vector<std::vector<float>> getImageDifferences(const cv::Mat &image) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        const size_t numSnapshots = getNumSnapshots();
        assert(numSnapshots > 0);

        Rotater rotater(getScanStep(), image);

        // Create vector to store RIDF values
        std::vector<std::vector<float>> differences(numSnapshots);
        for (auto &d : differences) {
            d.resize(rotater.max());
        }

        // Scan across image columns
        do {
            // Loop through snapshots
            for (size_t s = 0; s < numSnapshots; s++) {
                // Calculate difference
                const auto diff = calcSnapshotDifference(rotater.getImage(), m_ScratchMaskImage, s);
                differences[s][rotater.count()] = diff;
            }

            // **TODO**: Add this feature back in!
            // if (!m_ScratchMaskImage.empty()) {
            //     rollImage(m_ScratchMaskImage);
            // }
        } while (rotater.next());

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

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    filesystem::path getSnapshotPath(size_t index) const
    {
        return getSnapshotsPath() / getRouteDatabaseFilename(index);
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable cv::Mat m_ScratchMaskImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics