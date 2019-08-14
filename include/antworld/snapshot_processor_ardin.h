#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

// Libantworld includes
#include "snapshot_processor.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessorArdin
//----------------------------------------------------------------------------
//! OpenCV-based snapshot processor - uses OpenCV on CPU to process snapshots
//! In the manner described in Ardin, Webb (2016)
namespace BoBRobotics
{
namespace AntWorld
{
class SnapshotProcessorArdin : public SnapshotProcessor
{
public:
    SnapshotProcessorArdin(int displayScale, int intermediateWidth, int intermediateHeight,
                           int outputWidth, int outputHeight);

    //------------------------------------------------------------------------
    // SnapshotProcessor virtuals
    //------------------------------------------------------------------------
    //! Process input snapshot (probably at screen resolution)
    virtual void process(const cv::Mat &snapshot) override;

    //! Get processed snapshot - should be in a CV_8UC1 format image
    virtual const cv::Mat &getFinalSnapshot() const override{ return m_FinalSnapshot; }

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

    // CLAHE algorithm for histogram normalization
    cv::Ptr<cv::CLAHE> m_Clahe;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
