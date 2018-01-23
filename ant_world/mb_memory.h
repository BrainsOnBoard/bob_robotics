#pragma once

// Standard C++ includes
#include <tuple>

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
    std::tuple<unsigned int, unsigned int, unsigned int> present(float *inputData, unsigned int inputDataStep, bool reward);
};
