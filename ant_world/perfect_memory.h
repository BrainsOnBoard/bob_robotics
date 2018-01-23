#pragma once

// Standard C++ includes
#include <array>
#include <future>
#include <list>
#include <tuple>

// OpenCV includes
#include <opencv2/opencv.hpp>

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
    std::future<std::tuple<unsigned int, unsigned int, unsigned int>> present(const cv::Mat &snapshotFloat, bool train);

private:
    //------------------------------------------------------------------------
    // Private methods
    //------------------------------------------------------------------------
    std::tuple<unsigned int, unsigned int, unsigned int> presentThread(const cv::Mat &snapshotFloat, bool train);

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::list<std::array<float, Parameters::inputWidth * Parameters::inputHeight>> m_Snapshots;
};
