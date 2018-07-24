#pragma once

// Standard C++ includes
#include <array>
#include <list>
#include <tuple>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Antworld includes
#include "mb_params.h"

//----------------------------------------------------------------------------
// PerfectMemory
//----------------------------------------------------------------------------
class PerfectMemory
{
public:
    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    std::tuple<unsigned int, unsigned int, unsigned int> present(const cv::Mat &snapshotFloat, bool train);

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::list<std::array<float, MBParams::inputWidth * MBParams::inputHeight>> m_Snapshots;
};
