#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

namespace BoBRobotics
{
namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessor
//----------------------------------------------------------------------------
//! OpenCV-based snapshot processor - uses OpenCV on CPU to process snapshots
class SnapshotProcessorArdin
{
public:
    SnapshotProcessorArdin(int displayScale, int intermediateWidth, int intermediateHeight,
                           int outputWidth, int outputHeight);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // Process input snapshot (probably at screen resolution)
    void process(const cv::Mat &snapshot);

    const cv::Mat &getFinalSnapshot() const{ return m_FinalSnapshot; }
    const cv::Mat &getFinalSnapshotFloat() const{ return m_FinalSnapshotFloat; }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    // How much larger than intermediate image size is snapshot
    const int m_DisplayScale;

    // Dimensions of intermediate image
    const int m_IntermediateWidth;
    const int m_IntermediateHeight;

    // Dimensions of final output
    const int m_OutputWidth;
    const int m_OutputHeight;

    // Host OpenCV array to hold intermediate resolution greyscale snapshot
    cv::Mat m_IntermediateSnapshotGreyscale;

    // Host OpenCV array to hold final resolution greyscale snapshot
    cv::Mat m_FinalSnapshot;

    cv::Mat m_FinalSnapshotFloat;

    // CLAHE algorithm for histogram normalization
    cv::Ptr<cv::CLAHE> m_Clahe;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
