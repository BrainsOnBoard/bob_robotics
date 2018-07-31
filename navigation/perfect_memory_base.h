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
//#include "config.h"
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
      : NavigationBase(unwrapRes, outputPath)
      , m_ScanStep(scanStep)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual size_t getNumSnapshots() const = 0;
    virtual const cv::Mat &getSnapshot(size_t index) const = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void loadSnapshots(bool resizeImages = false)
    {
        loadSnapshotsFromPath(getOutputPath(), resizeImages);
    }

    void loadSnapshotsFromPath(const filesystem::path &routePath, bool resizeImages = false)
    {
        for(size_t i = 0;;i++) {
            const auto filename = routePath / getRouteDatabaseFilename(i);
            if(!filename.exists()) {
                break;
            }

            // Load image
            cv::Mat image = cv::imread(filename.str(), cv::IMREAD_GRAYSCALE);
            assert(image.type() == CV_8UC1);
            const auto &unwrapRes = getUnwrapResolution();
            if (resizeImages) {
                cv::resize(image, image, unwrapRes);
            } else {
                assert(image.cols == unwrapRes.width);
                assert(image.rows == unwrapRes.height);
            }

            // Add snapshot
            addSnapshot(image);
        }
    }

    virtual void train(const cv::Mat &image) override
    {
        const auto &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);
        assert(image.type() == CV_8UC1);

        // Add snapshot
        addSnapshot(image);
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
        float minDifferenceSquared = std::numeric_limits<float>::max();
        int bestCol = 0;
        size_t bestSnapshot = std::numeric_limits<size_t>::max();
        const size_t numSnapshots = getNumSnapshots();
        for(int i = 0; i < m_ScratchRollImage.cols; i += m_ScanStep) {
            // Loop through snapshots
            for(size_t s = 0; s < numSnapshots; s++) {
                // Calculate difference
                const float differenceSquared = calcSnapshotDifferenceSquared(m_ScratchRollImage, m_ScratchMaskImage, s);

                // If this is an improvement - update
                if(differenceSquared < minDifferenceSquared) {
                    minDifferenceSquared = differenceSquared;
                    bestCol = i;
                    bestSnapshot = s;
                }
            }

            // Roll image and corresponding mask left by m_ScanStep
            rollImage(m_ScratchRollImage, m_ScanStep);
            if(!m_ScratchMaskImage.empty()) {
                rollImage(m_ScratchMaskImage, m_ScanStep);
            }
        }

        // If best column is more than 180 degrees away, flip
        if(bestCol > (unwrapRes.width / 2)) {
            bestCol -= unwrapRes.width;
        }

        // Convert column into angle
        const radian_t bestAngle = units::make_unit<turn_t>((double)bestCol / (double)unwrapRes.width);

        // Return result
        return std::make_tuple(bestAngle, bestSnapshot, minDifferenceSquared);
    }

protected:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    // Add a snapshot to memory and return its index
    virtual size_t addSnapshot(const cv::Mat &image) = 0;

    // Calculate difference between memory and snapshot with index
    virtual float calcSnapshotDifferenceSquared(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot) const = 0;

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    filesystem::path getSnapshotPath(size_t index) const
    {
        return getOutputPath() / getRouteDatabaseFilename(index);
    }

    //------------------------------------------------------------------------
    // Private static methods
    //------------------------------------------------------------------------
    // 'Rolls' an image scanStep to the left
    static void rollImage(cv::Mat &image, unsigned int scanStep)
    {
        // Buffer to hold scanStep of pixels
        uint8_t *rollBuffer = new uint8_t[scanStep];

        // Loop through rows
        for(int y = 0; y < image.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = image.ptr(y);

            // Copy scanStep pixels at left hand size of row into buffer
            std::copy_n(rowPtr, scanStep, rollBuffer);

            // Copy rest of row back over pixels we've copied to buffer
            std::copy_n(rowPtr + scanStep, image.cols - scanStep, rowPtr);

            // Copy buffer back into row
            std::copy(rollBuffer, &rollBuffer[scanStep], rowPtr + (image.cols - scanStep));
        }

        // ** YUCK **
        delete rollBuffer;
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const unsigned int m_ScanStep;

    mutable cv::Mat m_ScratchMaskImage;
    mutable cv::Mat m_ScratchRollImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics