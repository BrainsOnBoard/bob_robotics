#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// Standard C++ includes
#include <iostream>
#include <limits>
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

//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryBase
//------------------------------------------------------------------------
class PerfectMemoryBase
  : public NavigationBase
{
private:
    class BestMatchingSnapshotProcessor
    {
        public:
        inline void operator()(float difference, int col, size_t snapshot)
        {
            if(difference < m_MinDifference) {
                m_MinDifference = difference;
                m_BestCol = col;
                m_BestSnapshot = snapshot;
            }
        }

        inline auto result(const cv::Size &unwrapRes)
        {
            // If best column is more than 180 degrees away, flip
            if(m_BestCol > (unwrapRes.width / 2)) {
                m_BestCol -= unwrapRes.width;
            }

            // Convert column into angle
            const radian_t bestAngle = units::make_unit<turn_t>((double) m_BestCol / (double) unwrapRes.width);

            // Bundle up result as a tuple
            return std::make_tuple(bestAngle, m_BestSnapshot, m_MinDifference);
        }

        private:
        float m_MinDifference = std::numeric_limits<float>::infinity();
        int m_BestCol;
        size_t m_BestSnapshot;
    };


    class RIDFValueLogger
      : public BestMatchingSnapshotProcessor
    {
        public:
        inline void operator()(float difference, int col, size_t snapshot)
        {
            BestMatchingSnapshotProcessor::operator()(difference, col, snapshot);
            m_Differences.push_back(difference);
        }

        inline auto result(const cv::Size &unwrapRes)
        {
            const auto res = BestMatchingSnapshotProcessor::result(unwrapRes);
            return std::make_tuple(std::get<0>(res), std::get<1>(res), std::get<2>(res), std::move(m_Differences));
        }

        private:
        std::vector<float> m_Differences;
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
        for(int i = 0; i < m_ScratchRollImage.cols; i += scanStep) {
            // Loop through snapshots
            for(size_t s = 0; s < numSnapshots; s++) {
                // Calculate difference
                const float difference = calcSnapshotDifference(m_ScratchRollImage, m_ScratchMaskImage, s);
                processor(difference, i, s);
            }

            // Roll image and corresponding mask left by scanStep
            rollImage(m_ScratchRollImage);
            if(!m_ScratchMaskImage.empty()) {
                rollImage(m_ScratchMaskImage);
            }
        }

        // Return result
        return processor.result(unwrapRes);
    }

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
        return runRIDF(image, BestMatchingSnapshotProcessor());
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