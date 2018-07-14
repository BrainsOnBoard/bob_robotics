#include "perfect_memory.h"

// Standard C++ includes
#include <functional>
#include <numeric>

// Standard C includes
#include <cmath>

//----------------------------------------------------------------------------
// PerfectMemory
//----------------------------------------------------------------------------
std::tuple<unsigned int, unsigned int, unsigned int> PerfectMemory::present(const cv::Mat &snapshotFloat, bool train)
{
    // Loop through snapshots
    float minSquareFamiliarity = std::numeric_limits<float>::max();
    for(const auto &s : m_Snapshots) {
        // Calculate sum of square difference between two snapshots
        const float squareSimilarity = std::inner_product(s.cbegin(), s.cend(), snapshotFloat.data, 0.0f, std::plus<float>(),
                                                          [](float a, float b)
                                                          {
                                                              return (a - b) * (a - b);
                                                          });

        // Update minimum
        minSquareFamiliarity = std::min(squareSimilarity, minSquareFamiliarity);
    }

    if(train) {
        m_Snapshots.emplace_back();
        std::copy_n(snapshotFloat.data, Parameters::inputWidth * Parameters::inputHeight, std::begin(m_Snapshots.back()));
    }

    // Return tuple of similarity
    return std::make_tuple(0, 0, (int)(sqrt(minSquareFamiliarity) * 0.005f));
}