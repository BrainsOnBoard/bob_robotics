#include "mb_memory_hog.h"

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <random>

// Standard C includes
#include <cmath>

// CLI11 includes
#include "third_party/CLI11.hpp"

// BoB robotics includes
#include "common/timer.h"
#include "genn_utils/connectors.h"

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::angle;
using namespace units::math;

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
MBMemoryHOG::MBMemoryHOG()
    :   MBMemory(MBParamsHOG::numPN, MBParamsHOG::numKC, MBParamsHOG::numEN, MBParamsHOG::numPNSynapsesPerKC,
                 MBParamsHOG::inputWidth, MBParamsHOG::inputHeight,
                 MBParamsHOG::tauD, MBParamsHOG::kcToENWeight, MBParamsHOG::dopamineStrength,
                 MBParamsHOG::rewardTimeMs, MBParamsHOG::presentDurationMs, MBParamsHOG::timestepMs,
                 "mb_memory_hog"),
        m_SobelX(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_32FC1), m_SobelY(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_32FC1),
        m_PixelOrientations(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_MAKETYPE(CV_32F, MBParamsHOG::hogNumOrientations)),
        m_HOGFeatures(MBParamsHOG::hogNumRFY, MBParamsHOG::hogNumRFX, CV_MAKETYPE(CV_32F, MBParamsHOG::hogNumOrientations)),
        m_PNToKCTauSyn(3.0f), m_PNTauM(10.0), m_PNC(1.0)
{
    std::cout << "HOG feature vector length:" << MBParamsHOG::hogFeatureSize << std::endl;

    // Build orientation vectors
    radian_t orient = 90_deg;
    for(auto &d : m_HOGDirections) {
        d[0] = sin(orient);
        d[1] = cos(orient);
        orient += 60_deg;
    }

    // Get pointers to EGPs
    m_GGGNToKC = getSLM().getScalar<float>("gggnToKC");
    m_GKCToGGN = getSLM().getScalar<float>("gkcToGGN");
    m_GPNToKC = getSLM().getScalar<float>("gpnToKC");
    m_VMidGGNToKC = getSLM().getScalar<float>("VmidggnToKC");
    m_VSlopeGGNToKC = getSLM().getScalar<float>("VslopeggnToKC");
    m_VThreshGGNToKC = getSLM().getScalar<float>("VthreshggnToKC");
    m_IExtScalePN = getSLM().getScalar<float>("IextScalePN");
    m_VThreshPN = getSLM().getScalar<float>("VthreshPN");
    m_ExpTCPN = getSLM().getScalar<float>("ExpTCPN");
    m_RmembranePN = getSLM().getScalar<float>("RmembranePN");
    m_ExpDecaypnToKC = getSLM().getScalar<float>("expDecaypnToKC");
    m_InitPNToKC = getSLM().getScalar<float>("initpnToKC");

    // Get pointers to state variables
    m_IExtPN = getSLM().getArray<float>("IextPN");
    m_VGGN = getSLM().getArray<float>("VGGN");
    m_InSynGGNToKC = getSLM().getArray<float>("inSynggnToKC");
    m_CKCToEN = getSLM().getArray<float>("ckcToEN");
    m_VEN = getSLM().getArray<float>("VEN");

    // Set initial EGP values from namespace
    *getGGNToKCWeight() = MBParamsHOG::ggnToKCWeight;
    *getKCToGGNWeight() = MBParamsHOG::kcToGGNWeight;
    *getPNToKC() = MBParamsHOG::pnToKCWeight;

    *getPNInputCurrentScale() = MBParamsHOG::inputCurrentScale;
    *getPNVthresh() = MBParamsHOG::pnVthresh;

    *getGGNToKCVMid() = MBParamsHOG::ggnToKCVMid;
    *getGGNToKCVslope() = MBParamsHOG::ggnToKCVslope;
    *getGGNToKCVthresh() = MBParamsHOG::ggnToKCVthresh;
}
//----------------------------------------------------------------------------
void MBMemoryHOG::write(cv::FileStorage& fs) const
{
    MBMemory::write(fs);

    fs << "pn" << "{";
    fs << "inputCurrentScale" << *m_IExtScalePN;
    fs << "vThresh" << *m_VThreshPN;
    fs << "cm" << m_PNC;
    fs << "tauM" << m_PNTauM;
    fs << "}";

    fs << "ggnToKC" << "{";
    fs << "weight" << *m_GGGNToKC;
    fs << "vMid" << *m_VMidGGNToKC;
    fs << "vSlope" << *m_VSlopeGGNToKC;
    fs << "vThresh" << *m_VThreshGGNToKC;
    fs << "}";

    fs << "kcToGGC" << "{";
    fs << "weight" << *m_GKCToGGN;
    fs << "}";

    fs << "pnToKC" << "{";
    fs << "weight" << *m_GPNToKC;
    fs << "tauSyn" << m_PNToKCTauSyn;
    fs << "}";
}
//----------------------------------------------------------------------------
void MBMemoryHOG::read(const cv::FileNode &node)
{
    MBMemory::read(node);

    const auto &pn = node["pn"];
    if(pn.isMap()) {
        cv::read(pn["inputCurrentScale"], *m_IExtScalePN, *m_IExtScalePN);
        cv::read(pn["vThresh"], *m_VThreshPN, *m_VThreshPN);
        cv::read(pn["cm"], m_PNC, m_PNC);
        cv::read(pn["tauM"], m_PNTauM, m_PNTauM);
    }

    const auto &ggnToKC = node["ggnToKC"];
    if(ggnToKC.isMap()) {
        cv::read(ggnToKC["weight"], *m_GGGNToKC, *m_GGGNToKC);
        cv::read(ggnToKC["vMid"], *m_VMidGGNToKC, *m_VMidGGNToKC);
        cv::read(ggnToKC["vSlope"], *m_VSlopeGGNToKC, *m_VSlopeGGNToKC);
        cv::read(ggnToKC["vThresh"], *m_VThreshGGNToKC, *m_VThreshGGNToKC);
    }

    const auto &kcToGGC = node["kcToGGC"];
    if(kcToGGC.isMap()) {
        cv::read(kcToGGC["weight"], *m_GKCToGGN, *m_GKCToGGN);
    }

    const auto &pnToKC = node["pnToKC"];
    if(pnToKC.isMap()) {
        cv::read(pnToKC["weight"], *m_GPNToKC, *m_GPNToKC);
        cv::read(pnToKC["tauSyn"], m_PNToKCTauSyn, m_PNToKCTauSyn);
    }
}
//----------------------------------------------------------------------------
void MBMemoryHOG::addCLIArguments(CLI::App &app)
{
    app.add_option("--ggn-to-kc-weight", *getGGNToKCWeight(), "GGN to KC weight", true);
    app.add_option("--kc-to-ggn-weight", *getKCToGGNWeight(), "KC to GGN weight", true);
    app.add_option("--ggn-to-kc-vmid", *getGGNToKCVMid(), "GGN to KC sigmoid midpoint", true);
    app.add_option("--ggn-to-kc-vslope", *getGGNToKCVslope(), "GGN to KC sigmoid slope", true);
    app.add_option("--ggn-to-kc-vthresh", *getGGNToKCVthresh(), "GGN to KC activation threshold", true);
}
//----------------------------------------------------------------------------
void MBMemoryHOG::initPresent(unsigned long long duration) const
{
    // Apply Sobel operator to image
    cv::Sobel(getSnapshotFloat(), m_SobelX, CV_32F, 1, 0, 1);
    cv::Sobel(getSnapshotFloat(), m_SobelY, CV_32F, 0, 1, 1);

    // At each pixel, take dot product of vector formed from x and y sobel operator and each direction vector
    typedef cv::Vec<float, MBParamsHOG::hogNumOrientations> PixelFeatures;
    static_assert(sizeof(PixelFeatures) == (MBParamsHOG::hogNumOrientations * sizeof(float)));
    std::transform(m_SobelX.begin<float>(), m_SobelX.end<float>(), m_SobelY.begin<float>(), m_PixelOrientations.begin<PixelFeatures>(),
                   [this](float x, float y)
                   {
                       PixelFeatures pix;
                       for(size_t d = 0; d < MBParamsHOG::hogNumOrientations; d++) {
                           pix[d] = std::abs((x * m_HOGDirections[d][0]) + (y * m_HOGDirections[d][1]));
                       }
                       return pix;
                   });

    // Loop through receptive fields
    auto hogOut = m_HOGFeatures.begin<PixelFeatures>();
    for(int rfY = 0; rfY <= (MBParamsHOG::inputHeight - MBParamsHOG::hogRFSize); rfY += MBParamsHOG::hogRFStride) {
        for(int rfX = 0; rfX <= (MBParamsHOG::inputWidth - MBParamsHOG::hogRFSize); rfX += MBParamsHOG::hogRFStride) {
            // Get ROI into hog directions representing pixels within receptive field
            const cv::Mat rfInput(m_PixelOrientations, cv::Rect(rfX, rfY, MBParamsHOG::hogRFSize, MBParamsHOG::hogRFSize));

            // Sum all pixels within receptive field
            const cv::Scalar sum = cv::sum(rfInput);

            // Calculate the exponential of each receptive field response
            std::array<float, MBParamsHOG::hogNumOrientations> exponentials;
            std::transform(&sum[0], &sum[MBParamsHOG::hogNumOrientations], exponentials.begin(),
                [](float s){ return std::exp(s); });

            // Sum these to get softmax scaling factor
            const float scale = std::accumulate(exponentials.cbegin(), exponentials.cend(), 0.0f);

            // Fill in features with softmax of orientations at location
            PixelFeatures features;
            std::transform(exponentials.cbegin(), exponentials.cend(), &features[0],
                           [scale](float e){ return e / scale; });
            (*hogOut++) = features;
        }
    }

    // Set 'derived' extra global params
    *m_ExpTCPN = std::exp(-MBParamsHOG::timestepMs / m_PNTauM);
    *m_RmembranePN = m_PNTauM / m_PNC;
    *m_ExpDecaypnToKC = (float)std::exp(-MBParamsHOG::timestepMs / m_PNToKCTauSyn);
    *m_InitPNToKC = (float)(m_PNToKCTauSyn * (1.0 - std::exp(-MBParamsHOG::timestepMs / m_PNToKCTauSyn))) * (1.0 / MBParamsHOG::timestepMs);

    // Clear GGN voltage and reserve
    m_GGNVoltageHistory.clear();
    m_GGNVoltageHistory.reserve(duration);

    m_KCInhInSynHistory.clear();
    m_KCInhInSynHistory.reserve(duration);
}
//----------------------------------------------------------------------------
void MBMemoryHOG::beginPresent() const
{
    // Copy HOG features into external input current
    BOB_ASSERT(m_HOGFeatures.isContinuous());
    std::copy_n(reinterpret_cast<float*>(m_HOGFeatures.data), MBParamsHOG::hogFeatureSize, m_IExtPN);
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryHOG::endPresent() const
{
    std::fill_n(m_IExtPN, MBParamsHOG::numPN, 0.0f);

    // Copy external input current to device
    getSLM().pushVarToDevice("PN", "Iext");
}
//----------------------------------------------------------------------------
void MBMemoryHOG::recordAdditional() const
{
     // **NOTE** very sub-optimal as we only use first!
    getSLM().pullStateFromDevice("ggnToKC");
    getSLM().pullStateFromDevice("kcToEN");

    m_GGNVoltageHistory.push_back(m_VGGN[0]);
    m_KCInhInSynHistory.push_back(m_InSynGGNToKC[0]);
}
