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
void rasterPlot(unsigned int numNeurons, const MBMemoryHOG::Spikes &spikes, float timeAxisStep = 20.0f, float pointSize = 1.0f)
{
    if(spikes.empty()) {
        return;
    }

    // Create dummy widget
    // **TODO** handle different dt
    assert(MBParams::timestepMs == 1.0);
    const float width = spikes.back().first * pointSize;
    const float height = (float)numNeurons * pointSize;
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
        ImGui::GetWindowDrawList()->AddText(ImVec2(rasterLeft + (t * pointSize) - (tickDims.x * 0.5f), rasterTop + height + 2),
                                            IM_COL32(128, 128, 128, 255), tickText);
    }

    // Loop through timesteps during which spikes occured
    for(const auto &timestepSpikes : spikes) {
        // Loop through neuron ids which spiked this timestep
        for(unsigned int n : timestepSpikes.second) {
            // Calculate coordinate
            const float x = rasterLeft + (timestepSpikes.first * pointSize);
            const float y = rasterTop + ((float)n * pointSize);
            ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(x, y), ImVec2(x + pointSize, y + pointSize),
                                                      IM_COL32(255, 255, 255, 255));
        }
    }
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
    if(ImGui::Begin("Unused weights")) {
        // Plot record of unused weights
        ImGui::PlotLines("", m_UnusedWeightsData.data(), m_UnusedWeightsData.size(), 0, nullptr, 0,
                         MBParams::numKC * MBParams::numEN, ImVec2(0, 200));

        if(ImGui::Button("Clear")) {
            m_UnusedWeightsData.clear();
        }
        ImGui::End();
    }

    if(ImGui::Begin("PN spikes")) {
        rasterPlot(MBParams::numPN, m_Memory.getPNSpikes(), 50.0f);
        ImGui::End();
    }

    /*if(ImGui::Begin("KC spikes")) {
        rasterPlot(MBParams::numKC, m_Memory.getKCSpikes(), 0.1f);
        ImGui::End();
    }*/

}
//----------------------------------------------------------------------------
void MBHogUI::handleUITraining()
{
    // Add number of unused weights to vector
    m_UnusedWeightsData.push_back((float)m_Memory.getNumUnusedWeights());
}
//----------------------------------------------------------------------------
void MBHogUI::handleUITesting()
{
}