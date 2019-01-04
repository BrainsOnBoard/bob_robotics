#include "visual_navigation_ui.h"

// IMGUI
#include "third_party/imgui/imgui.h"

// Antworld includes
#include "mb_memory_hog.h"
#include "mb_params.h"

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
bool rasterPlot(unsigned int numNeurons, const MBMemoryHOG::Spikes &spikes, float yScale = 1.0f, float timeAxisStep = 50.0f)
{
    if(spikes.empty()) {
        return false;
    }

    // Create dummy widget
    // **TODO** handle different dt
    assert(MBParams::timestepMs == 1.0);
    const float width = spikes.back().first;
    const float height = (float)numNeurons * yScale;
    const float leftBorder = 20.0f;
    const float topBorder = 10.0f;
    ImGui::Dummy(ImVec2(width + leftBorder, height + topBorder));

    const auto windowPos = ImGui::GetWindowPos();
    const float rasterLeft = windowPos.x + leftBorder;
    const float rasterTop = windowPos.y + topBorder;

    // Y-axis
    ImGui::GetWindowDrawList()->AddLine(ImVec2(rasterLeft, rasterTop),
                                        ImVec2(rasterLeft, rasterTop + height),
                                        IM_COL32(128, 128, 128, 255));

    // X-axis
    ImGui::GetWindowDrawList()->AddLine(ImVec2(rasterLeft, rasterTop + height),
                                        ImVec2(rasterLeft + width, rasterTop + height),
                                        IM_COL32(128, 128, 128, 255));

    char tickText[32];
    for(float t = 0.0f; t < spikes.back().first; t += timeAxisStep) {

        snprintf(tickText, 32, "%0.1f", t);
        const auto tickDims = ImGui::CalcTextSize(tickText);
        ImGui::GetWindowDrawList()->AddText(ImVec2(rasterLeft + t - (tickDims.x * 0.5f), rasterTop + height + 2),
                                            IM_COL32(128, 128, 128, 255), tickText);
    }

    // Loop through timesteps during which spikes occured
    for(const auto &timestepSpikes : spikes) {
        // Loop through neuron ids which spiked this timestep
        for(unsigned int n : timestepSpikes.second) {
            // Calculate coordinate
            const float x = rasterLeft + timestepSpikes.first;
            const float y = rasterTop + ((float)n * yScale);
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(x, y), ImVec2(x + 1.0f, y + 1.0f),
                                                      IM_COL32(255, 255, 255, 255));
        }
    }

    return true;
}
}   // Anonymous namespace

//----------------------------------------------------------------------------
// MBHogUI
//----------------------------------------------------------------------------
MBHogUI::MBHogUI(MBMemoryHOG &memory)
    : m_Memory(memory)
{
}
//----------------------------------------------------------------------------
void MBHogUI::handleUI()
{
    if(ImGui::Begin("Statistics")) {
        // Plot record of unused weights
        ImGui::PlotLines("Unused weights", m_UnusedWeightsData.data(), m_UnusedWeightsData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));

        ImGui::PlotLines("Active PN", m_ActivePNData.data(), m_ActivePNData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));

        ImGui::PlotLines("Active KC", m_ActiveKCData.data(), m_ActiveKCData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));

        if(ImGui::Button("Clear")) {
            m_UnusedWeightsData.clear();
            m_ActivePNData.clear();
            m_ActiveKCData.clear();
        }
        ImGui::End();
    }

    if(ImGui::Begin("PN spikes")) {
        if(rasterPlot(MBParams::numPN, m_Memory.getPNSpikes())){
            ImGui::Text("%u/%u active", m_Memory.getNumActivePN(), MBParams::numPN);
        }
        ImGui::End();
    }

    if(ImGui::Begin("KC spikes")) {
        if(rasterPlot(MBParams::numKC, m_Memory.getKCSpikes(), 0.025f)){
            ImGui::Text("%u/%u active", m_Memory.getNumActiveKC(), MBParams::numKC);
        }
        ImGui::End();
    }

}
//----------------------------------------------------------------------------
void MBHogUI::handleUITraining()
{
    // Add number of unused weights to vector
    m_UnusedWeightsData.push_back((float)m_Memory.getNumUnusedWeights());

    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());
}
//----------------------------------------------------------------------------
void MBHogUI::handleUITesting()
{
    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());
}