#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// SnapshotProcessorHOG
//----------------------------------------------------------------------------
// OpenCV-based snapshot processor - uses OpenCV  on CPU to process snapshots
class SnapshotProcessorHOG
{
public:
    SnapshotProcessorHOG(unsigned int displayScale, unsigned int intermediateWidth, unsigned int intermediateHeight);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // Process input snapshot (probably at screen resolution)
    void process(const cv::Mat &snapshot);

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
    
    cv::HOGDescriptor m_HOG;
    cv::Mat m_SnapshotGreyscale;
    cv::Mat m_FinalSnapshotFloat;
    std::vector<float> m_SnapshotDescriptor;
    
    // How much larger than intermediate image size is snapshot
    /*const unsigned int m_DisplayScale;

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
    cv::Ptr<cv::CLAHE> m_Clahe;*/
};