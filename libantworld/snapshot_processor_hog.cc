#include "snapshot_processor_hog.h"

//----------------------------------------------------------------------------
// SnapshotProcessorHOG
//----------------------------------------------------------------------------
SnapshotProcessorHOG::SnapshotProcessorHOG(unsigned int displayScale, unsigned int intermediateWidth, unsigned int intermediateHeight)
:   m_DisplayScale(displayScale), m_IntermediateWidth(intermediateWidth), m_IntermediateHeight(intermediateHeight),
    m_SnapshotGreyscale(intermediateHeight, intermediateWidth, CV_8UC1), m_SnapshotDescriptor(720)
{
    // Configure HOG features
    m_HOG.winSize = cv::Size(intermediateWidth, intermediateHeight); 
    m_HOG.blockSize = cv::Size(10, 10);
    m_HOG.blockStride = cv::Size(10, 10);
    m_HOG.cellSize = cv::Size(10, 10);
    m_HOG.nbins = 8;
}
//----------------------------------------------------------------------------
void SnapshotProcessorHOG::process(const cv::Mat &snapshot)
{
    // Check snapshot is expected size
    assert((unsigned int)snapshot.rows == m_IntermediateHeight * m_DisplayScale);
    assert((unsigned int)snapshot.cols == m_IntermediateWidth * m_DisplayScale);

    // Calculate start and end offset of resize kernel
    const unsigned int kernelStart = m_DisplayScale / 4;
    const unsigned int kernelEnd = m_DisplayScale - kernelStart;
    const unsigned int kernelPixels = kernelStart * m_DisplayScale;

    // Loop through intermediate image rows
    // **NOTE** this technique for downsampling the image is taken from the Matlab
    // and MASSIVELY improves performance over standard cv::imresize algorithms
    for(unsigned int y = 0; y < m_IntermediateHeight; y++) {
        // Calculate rows averaging kernel should operate over
        const unsigned int kernelStartY = (y * m_DisplayScale) + kernelStart;
        const unsigned int kernelEndY = (y * m_DisplayScale) + kernelEnd;

        // Loop through intermediate image columns
        for(unsigned int x = 0; x < m_IntermediateWidth; x++) {
            // Calculate columns averaging kernel should operate over
            const unsigned int kernelStartX = (x * m_DisplayScale) + kernelStart;
            const unsigned int kernelEndX = (x * m_DisplayScale) + kernelEnd;

            // Loop over snapshot pixels in kernel and sum their green components
            unsigned int sum = 0;
            for(unsigned i = kernelStartY; i < kernelEndY; i++) {
                for(unsigned int j = kernelStartX; j < kernelEndX; j++) {
                    sum += snapshot.at<cv::Vec3b>(i, j)[1];
                }
            }

            // Divide sum by number of pixels in kernel to compute average and write to intermediate snapshot
            m_SnapshotGreyscale.at<uint8_t>(y, x) = sum / kernelPixels;
        }
    }
    
    // Compute descriptor
    m_HOG.compute(m_SnapshotGreyscale, m_SnapshotDescriptor);
    assert(m_SnapshotDescriptor.size() == 720);
    
    // Create an OpenCV header for it
    m_FinalSnapshotFloat = cv::Mat(cv::Size(720, 1), CV_32FC1, m_SnapshotDescriptor.data());
}