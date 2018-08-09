#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// Standard C++ includes
#include <iostream>
#include <limits>
#include <tuple>

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

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
class PerfectMemoryBase
  : public NavigationBase
{
public:
    PerfectMemoryBase(const cv::Size unwrapRes, const unsigned int scanStep = 1,
                      const filesystem::path outputPath = "snapshots")
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
    virtual void train(const cv::Mat &image, bool saveImage) override
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

    std::tuple<radian_t, size_t, float> getHeading(const cv::Mat &image) const
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Clone mask and image so they can be rolled inplace
        getMaskImage().copyTo(m_ScratchMaskImage);
        image.copyTo(m_ScratchRollImage);

        // Scan across image columns
        float minDifference = std::numeric_limits<float>::max();
        int bestCol = 0;
        size_t bestSnapshot = std::numeric_limits<size_t>::max();
        const size_t numSnapshots = getNumSnapshots();
        const size_t scanStep = getScanStep();
        for(int i = 0; i < m_ScratchRollImage.cols; i += scanStep) {
            // Loop through snapshots
            for(size_t s = 0; s < numSnapshots; s++) {
                // Calculate difference
                const float difference = calcSnapshotDifference(m_ScratchRollImage, m_ScratchMaskImage, s);

                // If this is an improvement - update
                if(difference < minDifference) {
                    minDifference = difference;
                    bestCol = i;
                    bestSnapshot = s;
                }
            }

            // Roll image and corresponding mask left by scanStep
            rollImage(m_ScratchRollImage);
            if(!m_ScratchMaskImage.empty()) {
                rollImage(m_ScratchMaskImage);
            }
        }

        // If best column is more than 180 degrees away, flip
        if(bestCol > (unwrapRes.width / 2)) {
            bestCol -= unwrapRes.width;
        }

        // Convert column into angle
        const radian_t bestAngle = units::make_unit<turn_t>((double)bestCol / (double)unwrapRes.width);

        // Return result
        return std::make_tuple(bestAngle, bestSnapshot, minDifference);
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