#include "visual_navigation_ui.h"

// Standard C++ includes
#include <fstream>

// IMGUI
#include "imgui.h"

// Antworld includes
#include "mb_memory_ardin.h"
#include "mb_memory_hog.h"
#include "mb_params_ardin.h"
#include "mb_params_hog.h"

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

    const float width = spikes.back().first;
    const float height = (float)numNeurons * yScale;
    constexpr float leftBorder = 20.0f;
    constexpr float topBorder = 20.0f;

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
bool hogPlot(const cv::Mat &features, const std::array<cv::Vec2f, MBParamsHOG::hogNumOrientations> &directions, float drawScale)
{
    // Calculate number of cells
    const size_t numCellX = features.cols;
    const size_t numCellY = features.rows;
    constexpr float leftBorder = 40.0f;
    constexpr float topBorder = 50.0f;

    // Calcualate render size of hog cells
    const float halfDrawScale = 0.5f * drawScale;

    // Create dummy widget to correctly layout window
    const float height = (drawScale * numCellY) + topBorder;
    ImGui::Dummy(ImVec2((drawScale * numCellX) + leftBorder, height));

    const auto windowPos = ImGui::GetWindowPos();
    const float hogLeft = windowPos.x + leftBorder;
    const float hogTop = windowPos.y + topBorder;

    // Loop through cells
    for(size_t x = 0; x < numCellX; x++) {
        for(size_t y = 0; y < numCellY; y++) {
            const float drawX = hogLeft + (drawScale * (float)x);
            const float drawY = hogTop + (drawScale * (float)y);

            // Draw cell border
            ImGui::GetWindowDrawList()->AddRect(ImVec2(drawX - halfDrawScale, drawY - halfDrawScale),
                                                ImVec2(drawX + halfDrawScale, drawY + halfDrawScale),
                                                IM_COL32(128, 128, 128, 255));

            // Get magnitude of features in cell
            const auto cellFeatures = features.at<cv::Vec<float, MBParamsHOG::hogNumOrientations>>(y, x);

            for(size_t b = 0; b < MBParamsHOG::hogNumOrientations; b++) {
                // Skip zero-strength gradients
                if(cellFeatures[b] == 0.0f) {
                    continue;
                }

                const float lineLength = cellFeatures[b] * 10.0f;
                ImGui::GetWindowDrawList()->AddLine(ImVec2(drawX - (directions[b][0] * lineLength), drawY - directions[b][1] * lineLength),
                                                    ImVec2(drawX + (directions[b][0] * lineLength), drawY + directions[b][1] * lineLength),
                                                    IM_COL32(255, 255, 255, 255));
            }
        }
    }

    return true;
}
}   // Anonymous namespace


//----------------------------------------------------------------------------
// MBUI
//----------------------------------------------------------------------------
MBUI::MBUI(MBMemory &memory, const std::string &filename, unsigned int numPN, unsigned int numKC)
    : m_Memory(memory), m_Filename(filename), m_NumPN(numPN), m_NumKC(numKC)
{
    // Load memory config
    cv::FileStorage configFile(filename.c_str(), cv::FileStorage::READ);
    if(configFile.isOpened()) {
        m_Memory.read(configFile["config"]);
    }
}
//----------------------------------------------------------------------------
void MBUI::handleUI()
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

        ImGui::PlotLines("Num EN", m_NumENData.data(), m_NumENData.size(), 0, nullptr,
                         FLT_MAX, FLT_MAX, ImVec2(0, 50));
        if(!m_NumENData.empty()) {
            ImGui::Text("%.0f min, %.0f max", *std::min_element(m_NumENData.begin(), m_NumENData.end()),
                        *std::max_element(m_NumENData.begin(), m_NumENData.end()));
        }

        if(ImGui::Button("Clear")) {
            m_UnusedWeightsData.clear();
            m_ActivePNData.clear();
            m_ActiveKCData.clear();
            m_NumENData.clear();

            handleUIClear();
        }
    }
    ImGui::End();

    if(ImGui::Begin("PN spikes")) {
        if(rasterPlot(m_NumPN, m_Memory.getPNSpikes(), *m_Memory.getPresentDurationMs())){
            ImGui::Text("%u/%u active", m_Memory.getNumActivePN(), m_NumPN);
            ImGui::Text("%u spikes", m_Memory.getNumPNSpikes());
        }
    }
    ImGui::End();

    if(ImGui::Begin("KC spikes")) {
        if(rasterPlot(m_NumKC, m_Memory.getKCSpikes(), *m_Memory.getPresentDurationMs(), 0.025f)){
            ImGui::Text("%u/%u active", m_Memory.getNumActiveKC(), m_NumKC);
            ImGui::Text("%u spikes", m_Memory.getNumKCSpikes());
        }
    }
    ImGui::End();

    if(ImGui::Begin("EN spikes")) {
        if(rasterPlot(1, m_Memory.getENSpikes(), *m_Memory.getPresentDurationMs())){
            ImGui::Text("%u spikes", m_Memory.getNumENSpikes());
        }
    }
    ImGui::End();

    if(ImGui::Begin("MB parameters")) {
        handleUIMBProperties();

        if(ImGui::TreeNode("KC->EN")) {
            ImGui::SliderFloat("Dopamine strength", m_Memory.getKCToENDopamineStrength(), 0.0f, 1.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("Simulation")) {
            ImGui::SliderFloat("Reward time", m_Memory.getRewardTimeMs(), 0.0f, 200.0f);
            ImGui::SliderFloat("Present duration", m_Memory.getPresentDurationMs(), 0.0f, 200.0f);
            ImGui::TreePop();
        }

        if(ImGui::Button("Save")) {
            cv::FileStorage configFile(m_Filename, cv::FileStorage::WRITE);
            configFile << "config" << "{";
            m_Memory.write(configFile);
            configFile << "}";
        }
    }
    ImGui::End();

}
//----------------------------------------------------------------------------
void MBUI::handleUITraining()
{
    // Add number of unused weights to vector
    m_UnusedWeightsData.push_back((float)m_Memory.getNumUnusedWeights());

    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());
    m_NumENData.push_back((float)m_Memory.getNumENSpikes());
}
//----------------------------------------------------------------------------
void MBUI::handleUITesting()
{
    // Add number of active cells to vector
    m_ActivePNData.push_back((float)m_Memory.getNumActivePN());
    m_ActiveKCData.push_back((float)m_Memory.getNumActiveKC());
    m_NumENData.push_back((float)m_Memory.getNumENSpikes());
}
//----------------------------------------------------------------------------
void MBUI::saveLogs(const std::string &filename)
{
    assert(m_ActiveKCData.size() == m_ActivePNData.size());

    std::cout << "Logging..." << std::endl;
    std::ofstream log(filename);

    log << "Num active PNs, Num active KCs" << std::endl;
    for(size_t i = 0; i < m_ActiveKCData.size(); i++) {
        log << m_ActivePNData[i] << ", " << m_ActiveKCData[i] << std::endl;
    }
}

//----------------------------------------------------------------------------
// MBArdinUI
//----------------------------------------------------------------------------
MBArdinUI::MBArdinUI(MBMemoryArdin &memory)
    : MBUI(memory, "mb_memory_ardin.yml", MBParamsArdin::numPN, MBParamsArdin::numKC)
{
}

//----------------------------------------------------------------------------
// MBHogUI
//----------------------------------------------------------------------------
MBHogUI::MBHogUI(MBMemoryHOG &memory)
    : MBUI(memory, "mb_memory_hog.yml", MBParamsHOG::numPN, MBParamsHOG::numKC)
{
}
//----------------------------------------------------------------------------
void MBHogUI::handleUI()
{
    if(ImGui::Begin("HOG features", nullptr, ImGuiWindowFlags_NoResize)) {
        if(hogPlot(getMemoryHOG().getHOGFeatures(), getMemoryHOG().getHOGDirections(), 60.0f)) {
        }
    }
    ImGui::End();


    if(ImGui::Begin("GGN activity")) {
        ImGui::PlotLines("Membrane\nvoltage", getMemoryHOG().getGGNVoltageHistory().data(), getMemoryHOG().getGGNVoltageHistory().size(), 0, nullptr,
                         -60.0f, -40.0f, ImVec2(0, 50));
        ImGui::PlotLines("Inh out", getMemoryHOG().getKCInhInSynHistory().data(), getMemoryHOG().getKCInhInSynHistory().size(), 0, nullptr,
                         -1.0f, 0.0f, ImVec2(0, 50));
    }
    ImGui::End();

    // Superclass
    MBUI::handleUI();
}
//----------------------------------------------------------------------------
void MBHogUI::handleUIMBProperties()
{
    if(ImGui::TreeNode("PN")) {
            ImGui::SliderFloat("InputCurrentScale", getMemoryHOG().getPNInputCurrentScale(), 0.0f, 1.0f, "%.4f");
            ImGui::SliderFloat("VThresh", getMemoryHOG().getPNVthresh(), -60.0f, 0.0f);
            ImGui::SliderFloat("TauM", getMemoryHOG().getPNTauM(), 1.0f, 50.0f);
            ImGui::SliderFloat("CM", getMemoryHOG().getPNC(), 1.0f, 50.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("GGN->KC")) {
            ImGui::SliderFloat("Weight", getMemoryHOG().getGGNToKCWeight(), -10.0f, 0.0f);

            ImGui::SliderFloat("VMid", getMemoryHOG().getGGNToKCVMid(), -60.0f, -20.0f);
            ImGui::SliderFloat("Vslope", getMemoryHOG().getGGNToKCVslope(), 1.0f, 4.0f);
            ImGui::SliderFloat("Vthresh", getMemoryHOG().getGGNToKCVthresh(), -60.0f, -20.0f);
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("KC->GGN")) {
            ImGui::SliderFloat("Weight", getMemoryHOG().getKCToGGNWeight(), 0.0f, 0.04f, "%.4f");
            ImGui::TreePop();
        }

        if(ImGui::TreeNode("PN->KC")) {
            ImGui::SliderFloat("Weight", getMemoryHOG().getPNToKC(), 0.0f, 0.5f);
            ImGui::SliderFloat("TauSyn", getMemoryHOG().getPNToKCTauSyn(), 1.0f, 20.0f);
            ImGui::TreePop();
        }
}
//----------------------------------------------------------------------------
void MBHogUI::handleUIClear()
{
    m_PeakGGNVoltage.clear();
}
//----------------------------------------------------------------------------
void MBHogUI::handleUITraining()
{
    // Superclass
    MBUI::handleUITraining();

    m_PeakGGNVoltage.push_back(*std::max_element(getMemoryHOG().getGGNVoltageHistory().begin(), getMemoryHOG().getGGNVoltageHistory().end()));
}
//----------------------------------------------------------------------------
void MBHogUI::handleUITesting()
{
    // Superclass
    MBUI::handleUITesting();

    m_PeakGGNVoltage.push_back(*std::max_element(getMemoryHOG().getGGNVoltageHistory().begin(), getMemoryHOG().getGGNVoltageHistory().end()));
}
//----------------------------------------------------------------------------
MBMemoryHOG &MBHogUI::getMemoryHOG()
{
    return dynamic_cast<MBMemoryHOG&>(getMemory());
}
