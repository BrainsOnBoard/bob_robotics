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
void rasterPlot(unsigned int numNeurons, const MBMemoryHOG::Spikes &spikes, float pointSize = 1.0f)
{
    if(spikes.empty()) {
        return;
    }

    // Create dummy widget
    // **TODO** handle different dt
    ImGui::Dummy(ImVec2(spikes.back().first * pointSize, (float)numNeurons * pointSize));

    const auto windowPos = ImGui::GetWindowPos();
    for(const auto &s : spikes) {
        for(unsigned int n : s.second) {
            const float x = windowPos.x + (spikes.back().first * pointSize);
            const float y = windowPos.y + ((float)n * pointSize);
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
        rasterPlot(MBParams::numPN, m_Memory.getPNSpikes());
        ImGui::End();
    }

    if(ImGui::Begin("KC spikes")) {
        rasterPlot(MBParams::numKC, m_Memory.getKCSpikes(), 0.1f);
        ImGui::End();
    }

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