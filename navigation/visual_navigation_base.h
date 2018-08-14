#pragma once

// Standard C includes
#include <cassert>
#include <cstdint>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "../common/image_database.h"

// Third-party includes
#include "../third_party/path.h"

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::VisualNavigationBase
//------------------------------------------------------------------------
//! The base class for visual navigation algorithms (currently just variations on perfect memory)
class VisualNavigationBase
{
public:
    VisualNavigationBase(const cv::Size unwrapRes, const unsigned int scanStep,
                         const filesystem::path outputPath = "snapshots")
      : m_UnwrapRes(unwrapRes)
      , m_ScanStep(scanStep)
      , m_SnapshotsPath(outputPath)
      , m_ScratchMaskImage(unwrapRes, CV_8UC1)
      , m_ScratchRollImage(unwrapRes, CV_8UC1)
    {}

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image, bool saveImage) = 0;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void loadSnapshot(const filesystem::path &snapshotPath, bool resizeImage = false)
    {
        if (!snapshotPath.exists()) {
            throw std::runtime_error("Path " + snapshotPath.str() + " does not exist");
        }

        // Load image
        cv::Mat image = cv::imread(snapshotPath.str(), cv::IMREAD_GRAYSCALE);
        assert(image.type() == CV_8UC1);
        if (resizeImage) {
            cv::resize(image, image, m_UnwrapRes);
        } else {
            assert(image.cols == m_UnwrapRes.width);
            assert(image.rows == m_UnwrapRes.height);
        }

        // Add snapshot
        train(image, false);
    }

    //! Load snapshots from default path
    void loadSnapshots(bool resizeImages = false)
    {
        loadSnapshots(m_SnapshotsPath, resizeImages);
    }

    //! Load snapshots from specified path
    void loadSnapshots(const filesystem::path &routePath, bool resizeImages = false)
    {
        if (!routePath.exists()) {
            throw std::runtime_error("Path " + routePath.str() + " does not exist");
        }

        for (size_t i = 0;; i++) {
            const auto filename = routePath / getRouteDatabaseFilename(i);
            try {
                loadSnapshot(filename, resizeImages);
            } catch (std::runtime_error &) {
                return;
            }
        }
    }

    //! Save a snapshot to disk
    void saveSnapshot(const size_t index, const cv::Mat &image)
    {
        cv::imwrite(getSnapshotPath(index).str(), image);
    }

    //! Set mask image (e.g. for masking part of robot)
    void setMaskImage(const std::string path)
    {
        m_MaskImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
        assert(m_MaskImage.cols == m_UnwrapRes.width);
        assert(m_MaskImage.rows == m_UnwrapRes.height);
        assert(m_MaskImage.type() == CV_8UC1);
    }

    //! Number of pixels for each scanning "step" when doing RIDF
    inline unsigned int getScanStep() const
    {
        return m_ScanStep;
    }

    //! Return mask image
    const cv::Mat &getMaskImage() const
    {
        return m_MaskImage;
    }

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const
    {
        return m_UnwrapRes;
    }

    //! Get output path for snapshots
    const filesystem::path &getSnapshotsPath() const
    {
        return m_SnapshotsPath;
    }

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    filesystem::path getSnapshotPath(const size_t index) const
    {
        return m_SnapshotsPath / getRouteDatabaseFilename(index);
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const cv::Size m_UnwrapRes;
    const unsigned int m_ScanStep;
    const filesystem::path m_SnapshotsPath;
    cv::Mat m_MaskImage;

    mutable cv::Mat m_ScratchMaskImage;
    mutable cv::Mat m_ScratchRollImage;

protected:
    //! 'Rolls' an image to the left
    void rollImage(cv::Mat &image) const
    {
        // Loop through rows
        for (int y = 0; y < image.rows; y++) {
            // Get pointer to start of row
            uint8_t *rowPtr = image.ptr(y);

            // Rotate row to left by m_ScanStep pixels
            std::rotate(rowPtr, rowPtr + m_ScanStep, rowPtr + image.cols);
        }
    }
}; // PerfectMemoryBase
} // Navigation
} // BoBRobotics