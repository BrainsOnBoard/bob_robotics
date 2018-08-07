#pragma once

// Standard C++ includes
#include <future>
#include <tuple>

// OpenCV includes
#include <opencv2/opencv.hpp>

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
class MBMemory
{
public:
    MBMemory();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
   std::tuple<unsigned int, unsigned int, unsigned int> present(const cv::Mat &snapshotFloat, bool train);

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    std::tuple<unsigned int, unsigned int, unsigned int> presentThread(float *inputData, unsigned int inputDataStep, bool reward);

#ifndef CPU_ONLY
    cv::cuda::GpuMat m_SnapshotFloatGPU;
#endif
};
