#include "mb_memory_ardin.h"


// Antworld includes
#include "mb_params_ardin.h"

using namespace BoBRobotics;

//----------------------------------------------------------------------------
// MBMemoryArdin
//----------------------------------------------------------------------------
MBMemoryArdin::MBMemoryArdin()
    :   MBMemory(MBParamsArdin::numPN, MBParamsArdin::numKC, MBParamsArdin::numEN, MBParamsArdin::numPNSynapsesPerKC,
                 MBParamsArdin::inputWidth, MBParamsArdin::inputHeight,
                 MBParamsArdin::tauD, MBParamsArdin::kcToENWeight, MBParamsArdin::dopamineStrength,
                 MBParamsArdin::rewardTimeMs, MBParamsArdin::presentDurationMs, MBParamsArdin::postStimuliDurationMs, MBParamsArdin::timestepMs,
                 "mb_memory_ardin"),
        m_SnapshotNormalizedFloat(MBParamsArdin::inputHeight, MBParamsArdin::inputWidth, CV_32FC1)
{
    // Get pointers to state variables
    m_IExtPN = getSLM().getArray<float>("IextPN");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::initPresent(unsigned long long) const
{
}
//----------------------------------------------------------------------------
void MBMemoryArdin::beginPresent(const cv::Mat &snapshotFloat) const
{
    // Normalise input
    cv::normalize(snapshotFloat, m_SnapshotNormalizedFloat);

    // Scale normalised input into external input current
    BOB_ASSERT(m_SnapshotNormalizedFloat.isContinuous());
    std::transform(m_SnapshotNormalizedFloat.begin<float>(), m_SnapshotNormalizedFloat.end<float>(), m_IExtPN,
                   [](float x){ return x * MBParamsArdin::inputCurrentScale; });

    // Copy to device
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::endPresent() const
{
    // Zero external input current
    std::fill_n(m_IExtPN, MBParamsArdin::numPN, 0.0f);

    // Copy external input current to device
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::recordAdditional() const
{
}
