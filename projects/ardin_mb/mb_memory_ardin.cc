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
                 MBParamsArdin::rewardTimeMs, MBParamsArdin::presentDurationMs, MBParamsArdin::timestepMs,
                 "mb_memory_ardin")
{
    // Get pointers to state variables
    m_IExtPN = getSLM().getArray<float>("IextPN");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::initPresent(unsigned long long duration) const
{
}
//----------------------------------------------------------------------------
void MBMemoryArdin::beginPresent() const
{
    BOB_ASSERT(getSnapshotFloat().isContinuous());
    std::copy_n(reinterpret_cast<float*>(getSnapshotFloat().data), MBParamsArdin::numPN, m_IExtPN);
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::endPresent() const
{
    std::fill_n(m_IExtPN, MBParamsArdin::numPN, 0.0f);

    // Copy external input current to device
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryArdin::recordAdditional() const
{
}
