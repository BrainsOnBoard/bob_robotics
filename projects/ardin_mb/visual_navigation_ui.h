#pragma once

// Standard C++ includes
#include <vector>

// Forward declarations
class MBMemoryHOG;

//----------------------------------------------------------------------------
// VisualNavigationUI
//----------------------------------------------------------------------------
class VisualNavigationUI
{
public:
    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual void handleUI(){}
    virtual void handleUITraining(){}
    virtual void handleUITesting(){}
};


//----------------------------------------------------------------------------
// MBHogUI
//----------------------------------------------------------------------------
class MBHogUI : public VisualNavigationUI
{
public:
    MBHogUI(MBMemoryHOG &memory);

    //----------------------------------------------------------------------------
    // VisualNavigationUI virtuals
    //----------------------------------------------------------------------------
    virtual void handleUI() override;
    virtual void handleUITraining() override;
    virtual void handleUITesting() override;

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    MBMemoryHOG &m_Memory;

    // Data for plotting unused weights
    std::vector<float> m_UnusedWeightsData;
    std::vector<float> m_ActivePNData;
    std::vector<float> m_ActiveKCData;
    std::vector<float> m_PeakGGNVoltage;
};