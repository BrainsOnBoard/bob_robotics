#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessor
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
//! OpenCV-based snapshot processor - uses OpenCV on CPU to process snapshots
class SnapshotProcessorSegmentSky
{
public:
    SnapshotProcessorSegmentSky(int outputWidth, int outputHeight);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // Process input snapshot (probably at screen resolution)
    void process(const cv::Mat &snapshot);

    const cv::Mat &getFinalSnapshot() const{ return m_FinalSnapshot; }

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
