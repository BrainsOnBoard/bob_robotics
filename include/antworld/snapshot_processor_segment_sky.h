#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

// Libantworld includes
#include "snapshot_processor.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessorSegmentSky
//----------------------------------------------------------------------------
//! OpenCV-based snapshot processor - uses OpenCV on CPU to bianrize image based on sky
namespace BoBRobotics
{
namespace AntWorld
{
class SnapshotProcessorSegmentSky : public SnapshotProcessor
{
public:
    SnapshotProcessorSegmentSky(int outputWidth, int outputHeight);

    //------------------------------------------------------------------------
    // SnapshotProcessor virtuals
    //------------------------------------------------------------------------
    //! Process input snapshot (probably at screen resolution)
    virtual void process(const cv::Mat &snapshot) override;

    //! Get processed snapshot - should be in a CV_8UC1 format image
    virtual const cv::Mat &getFinalSnapshot() const override{ return m_FinalSnapshot; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Dimensions of final output
    const cv::Size m_OutputSize;

    // Host OpenCV array to hold resized snapshot before it's thresholded
    cv::Mat m_IntermediateSnapshotColour;

    // Host OpenCV array to hold final resolution greyscale snapshot
    cv::Mat m_FinalSnapshot;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
