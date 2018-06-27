#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessor
//----------------------------------------------------------------------------
// OpenCV-based snapshot processor - uses OpenCV  on CPU to process snapshots
namespace BoBRobotics
{
namespace AntWorld
{
class SnapshotProcessorArdin
{
public:
    SnapshotProcessorArdin(unsigned int displayScale, unsigned int intermediateWidth, unsigned int intermediateHeight,
                           unsigned int outputWidth, unsigned int outputHeight);

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
    const unsigned int m_DisplayScale;

    // Dimensions of intermediate image
    const unsigned int m_IntermediateWidth;
    const unsigned int m_IntermediateHeight;

    // Dimensions of final output
    const unsigned int m_OutputWidth;
    const unsigned int m_OutputHeight;

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