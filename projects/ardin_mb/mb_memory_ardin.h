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
    virtual void initPresent(unsigned long long duration) override;
    virtual void beginPresent(const cv::Mat &snapshotFloat) override;
    virtual void endPresent() override;
    virtual void recordAdditional() override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    float *m_IExtPN;

    cv::Mat m_SnapshotNormalizedFloat;
};
