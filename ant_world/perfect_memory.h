#pragma once

// Standard C++ includes
#include <array>
#include <list>
#include <tuple>

// OpenCV includes
#include "../common/opencv.h"

// Antworld includes
#include "parameters.h"

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
    std::list<std::array<float, Parameters::inputWidth * Parameters::inputHeight>> m_Snapshots;
};
