#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/image_database.h"

// Third-party includes
#include "../third_party/path.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::NavigationBase
//------------------------------------------------------------------------
class NavigationBase {
public:
    NavigationBase(const cv::Size unwrapRes,
                   const filesystem::path outputPath = "snapshots")
      : m_UnwrapRes(unwrapRes)
      , m_OutputPath(outputPath)
      , m_ScratchMaskImage(unwrapRes, CV_8UC1)
      , m_ScratchRollImage(unwrapRes, CV_8UC1)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image, bool saveImage) = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void loadSnapshots(bool resizeImages = false)
    {
        loadSnapshotsFromPath(m_OutputPath, resizeImages);
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
            if (resizeImages) {
                cv::resize(image, image, m_UnwrapRes);
            } else {
                assert(image.cols == m_UnwrapRes.width);
                assert(image.rows == m_UnwrapRes.height);
            }

            // Add snapshot
            train(image, false);
        }
    }

    void saveSnapshot(const size_t index, const cv::Mat &image)
    {
        cv::imwrite(getSnapshotPath(index).str(), image);
    }

    void setMaskImage(const std::string path)
    {
        m_MaskImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
        assert(m_MaskImage.cols == m_UnwrapRes.width);
        assert(m_MaskImage.rows == m_UnwrapRes.height);
        assert(m_MaskImage.type() == CV_8UC1);
    }

    const cv::Mat &getMaskImage() const { return m_MaskImage; }
    const cv::Size &getUnwrapResolution() const { return m_UnwrapRes; }
    const filesystem::path &getOutputPath() const { return m_OutputPath; }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    filesystem::path getSnapshotPath(const size_t index) const
    {
        return m_OutputPath / getRouteDatabaseFilename(index);
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const cv::Size m_UnwrapRes;
    const filesystem::path m_OutputPath;
    cv::Mat m_MaskImage;

    mutable cv::Mat m_ScratchMaskImage;
    mutable cv::Mat m_ScratchRollImage;
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics