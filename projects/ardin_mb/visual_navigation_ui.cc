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
bool rasterPlot(unsigned int numNeurons, const MBMemoryHOG::Spikes &spikes, float verticalLineTime, float yScale = 1.0f, float timeAxisStep = 50.0f)
{
    if(spikes.empty()) {
        return false;
    }

    // **TODO** handle different dt
    assert(MBParams::timestepMs == 1.0);
    const float width = spikes.back().first;
    const float height = (float)numNeurons * yScale;
    constexpr float leftBorder = 20.0f;
    constexpr float topBorder = 10.0f;

    // Create dummy widget to correctly layout window
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

    // Stimuli end
    ImGui::GetWindowDrawList()->AddLine(ImVec2(rasterLeft + verticalLineTime, rasterTop),
                                        ImVec2(rasterLeft + verticalLineTime, rasterTop + height),
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
//----------------------------------------------------------------------------
bool hogPlot(const std::vector<float> &features, float drawScale)
{
    using namespace units::literals;
    using namespace units::angle;

    // Check feature size is correct
    assert(features.size() == MBParams::hogFeatureSize);

    // Precalculate sin and cos of each bin angle
    // **THINK** just how unpleasant would a constexpr template metaprogramming solution be here?
    constexpr degree_t oneBinAngle = 180.0_deg / MBParams::hogNumOrientations;
    double binCos[MBParams::hogNumOrientations];
    double binSin[MBParams::hogNumOrientations];
    for(size_t b = 0; b < MBParams::hogNumOrientations; b++) {
        const degree_t binAngle = (b * oneBinAngle) + (oneBinAngle * 0.5);
        binCos[b] = units::math::cos(binAngle);
        binSin[b] = units::math::sin(binAngle);
    }

    // Calculate number of cells
    constexpr size_t numCellX = MBParams::inputWidth / MBParams::hogCellSize;
    constexpr size_t numCellY = MBParams::inputHeight / MBParams::hogCellSize;
    constexpr float leftBorder = 40.0f;
    constexpr float topBorder = 10.0f;

    // Calcualate render size of hog cells
    const float drawCellSize = drawScale * (float)MBParams::hogCellSize;
    const float halfDrawCellSize = 0.5f * drawCellSize;

    // Create dummy widget to correctly layout window
    const float height = (drawCellSize * numCellY) + topBorder;
    ImGui::Dummy(ImVec2((drawCellSize * numCellX) + leftBorder, height));

    const auto windowPos = ImGui::GetWindowPos();
    const float hogLeft = windowPos.x + leftBorder;
    const float hogBottom = windowPos.y + height;

    // Loop through cells
    size_t i = 0;
    for(size_t y = 0; y < numCellY; y++) {
        for(size_t x = 0; x < numCellX; x++) {
            const float drawX = hogLeft + (drawCellSize * (float)x);
            const float drawY = hogBottom - (drawCellSize * (float)y);

            // Draw cell border
            ImGui::GetWindowDrawList()->AddRect(ImVec2(drawX - halfDrawCellSize, drawY - halfDrawCellSize),
                                                ImVec2(drawX + halfDrawCellSize, drawY + halfDrawCellSize),
                                                IM_COL32(128, 128, 128, 255));
            for(size_t b = 0; b < MBParams::hogNumOrientations; b++) {
                // Get gradient strength
                const float gradStrength = features[i++];

                // Skip zero-strength gradients
                if(gradStrength == 0.0f) {
                    continue;
                }

                const float lineLength = gradStrength * drawCellSize;
                ImGui::GetWindowDrawList()->AddLine(ImVec2(drawX - (binCos[b] * lineLength), drawY - binSin[b] * lineLength),
                                                    ImVec2(drawX + (binCos[b] * lineLength), drawY + binSin[b] * lineLength),
                                                    IM_COL32(255, 255, 255, 255));
            }
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
    // Load memory config
    cv::FileStorage configFile("mb_memory_hog.yml", cv::FileStorage::READ);
    if(configFile.isOpened()) {
        m_Memory.read(configFile["config"]);
    }
}
//----------------------------------------------------------------------------
void MBHogUI::handleUI()
{
    if(ImGui::Begin("Statistics")) {
        // Plot record of unused weights
        ImGui::PlotLines("Unused\nweights", m_UnusedWeightsData.data(), m_UnusedWeightsData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));

        ImGui::PlotLines("Active PN", m_ActivePNData.data(), m_ActivePNData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));
        if(!m_ActivePNData.empty()) {
            ImGui::Text("%.0f min, %.0f max", *std::min_element(m_ActivePNData.begin(), m_ActivePNData.end()),
                        *std::max_element(m_ActivePNData.begin(), m_ActivePNData.end()));
        }

        ImGui::PlotLines("Active KC", m_ActiveKCData.data(), m_ActiveKCData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));
        if(!m_ActiveKCData.empty()) {
            ImGui::Text("%.0f min, %.0f max", *std::min_element(m_ActiveKCData.begin(), m_ActiveKCData.end()),
                        *std::max_element(m_ActiveKCData.begin(), m_ActiveKCData.end()));
        }


        if(ImGui::Button("Clear")) {
            m_UnusedWeightsData.clear();
            m_ActivePNData.clear();
            m_ActiveKCData.clear();
            m_PeakGGNVoltage.clear();
        }
    }
    ImGui::End();

    if(ImGui::Begin("HOG features", nullptr, ImGuiWindowFlags_NoResize)) {
        if(hogPlot(m_Memory.getHOGFeatures(), 20.0f)) {
        }
    }
    ImGui::End();

    if(ImGui::Begin("PN spikes")) {
        if(rasterPlot(MBParams::numPN, m_Memory.getPNSpikes(), *m_Memory.getPresentDurationMs())){
            ImGui::Text("%u/%u active", m_Memory.getNumActivePN(), MBParams::numPN);
            ImGui::Text("%u spikes", m_Memory.getNumPNSpikes());
        }
    }
    ImGui::End();

    if(ImGui::Begin("KC spikes")) {
        if(rasterPlot(MBParams::numKC, m_Memory.getKCSpikes(), *m_Memory.getPresentDurationMs(), 0.025f)){
            ImGui::Text("%u/%u active", m_Memory.getNumActiveKC(), MBParams::numKC);
            ImGui::Text("%u spikes", m_Memory.getNumKCSpikes());
        }
    }
    ImGui::End();

    if(ImGui::Begin("GGN activity")) {
        ImGui::PlotLines("Membrane\nvoltage", m_Memory.getGGNVoltage().data(), m_Memory.getGGNVoltage().size(), 0, nullptr,
                         -60.0f, -40.0f, ImVec2(0, 50));
        ImGui::PlotLines("Inh out", m_Memory.getKCInhInSyn().data(), m_Memory.getKCInhInSyn().size(), 0, nullptr,
                         -1.0f, 0.0f, ImVec2(0, 50));
    }
    ImGui::End();

    if(ImGui::Begin("MB parameters")) {
        if(ImGui::TreeNode("PN")) {
            ImGui::SliderFloat("InputCurrentScale", m_Memory.getPNInputCurrentScale(), 0.0f, 10.0f, "%.4f");
            ImGui::SliderFloat("VThresh", m_Memory.getPNVthresh(), -60.0f, 0.0f);
            ImGui::SliderFloat("TauM", m_Memory.getPNTauM(), 1.0f, 50.0f);
            ImGui::SliderFloat("CM", m_Memory.getPNC(), 1.0f, 50.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("GGN->KC")) {
            ImGui::SliderFloat("Weight", m_Memory.getGGNToKCWeight(), -10.0f, 0.0f);

            ImGui::SliderFloat("VMid", m_Memory.getGGNToKCVMid(), -60.0f, -20.0f);
            ImGui::SliderFloat("Vslope", m_Memory.getGGNToKCVslope(), 1.0f, 4.0f);
            ImGui::SliderFloat("Vthresh", m_Memory.getGGNToKCVthresh(), -60.0f, -20.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("KC->GGN")) {
            ImGui::SliderFloat("Weight", m_Memory.getKCToGGNWeight(), 0.0f, 0.04f, "%.4f");
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("PN->KC")) {
            ImGui::SliderFloat("Weight", m_Memory.getPNToKC(), 0.0f, 0.5f);
            ImGui::SliderFloat("TauSyn", m_Memory.getPNToKCTauSyn(), 1.0f, 20.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("Simulation")) {
            ImGui::SliderFloat("Reward time", m_Memory.getRewardTimeMs(), 0.0f, 200.0f);
            ImGui::SliderFloat("Present duration", m_Memory.getPresentDurationMs(), 0.0f, 200.0f);
            ImGui::TreePop();
        }

        if(ImGui::Button("Save")) {
            cv::FileStorage configFile("mb_memory_hog.yml", cv::FileStorage::WRITE);
            m_Memory.write(configFile);
        }
    }
    ImGui::End();

}
//----------------------------------------------------------------------------
void MBHogUI::handleUITraining()
{
    // Add number of unused weights to vector
    m_UnusedWeightsData.push_back((float)m_Memory.getNumUnusedWeights());

    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());

    m_PeakGGNVoltage.push_back(*std::max_element(m_Memory.getGGNVoltage().begin(), m_Memory.getGGNVoltage().end()));
}
//----------------------------------------------------------------------------
void MBHogUI::handleUITesting()
{
    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());

    m_PeakGGNVoltage.push_back(*std::max_element(m_Memory.getGGNVoltage().begin(), m_Memory.getGGNVoltage().end()));
}