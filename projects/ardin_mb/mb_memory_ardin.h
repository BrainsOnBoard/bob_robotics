#pragma once

// Standard C++ includes
#include <array>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Ardin MB includes
#include "mb_memory.h"

//----------------------------------------------------------------------------
// MBMemoryArdin
//----------------------------------------------------------------------------
class MBMemoryArdin : public MBMemory
{
public:
    MBMemoryArdin();

protected:
    //------------------------------------------------------------------------
    // MBMemory virtuals
    //------------------------------------------------------------------------
    virtual void initPresent(unsigned long long duration) const override;
    virtual void beginPresent() const override;
    virtual void endPresent() const override;
    virtual void recordAdditional() const override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    float *m_IExtPN;
};
