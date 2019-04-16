#pragma once

// Standard C++ includes
#include <string>
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

    virtual void saveLogs(const std::string &){};
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

    virtual void saveLogs(const std::string &filename) override;
private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    MBMemoryHOG &m_Memory;
    // Data for plotting unused weights
    std::vector<float> m_UnusedWeightsData;
    std::vector<float> m_ActivePNData;
    std::vector<float> m_ActiveKCData;
    std::vector<float> m_NumENData;
    std::vector<float> m_PeakGGNVoltage;
};
