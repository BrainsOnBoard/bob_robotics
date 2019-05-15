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
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
unsigned int convertMsToTimesteps(double ms)
{
    return (unsigned int)std::round(ms / MBParamsHOG::timestepMs);
}
void record(double t, unsigned int spikeCount, unsigned int *spikes, MBMemoryHOG::Spikes &spikeOutput)
{
    // Add a new entry to the cache
    spikeOutput.emplace_back();

    // Fill in time
    spikeOutput.back().first = t;

    // Reserve vector to hold spikes
    spikeOutput.back().second.reserve(spikeCount);

    // Copy spikes into vector
    std::copy_n(spikes, spikeCount, std::back_inserter(spikeOutput.back().second));
}
}   // Anonymous namespace

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
MBMemoryHOG::MBMemoryHOG()
    :   Navigation::VisualNavigationBase(cv::Size(MBParamsHOG::inputWidth, MBParamsHOG::inputHeight)),
        m_SnapshotFloat(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_32FC1), m_SobelX(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_32FC1),
        m_SobelY(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_32FC1), m_PixelOrientations(MBParamsHOG::inputHeight, MBParamsHOG::inputWidth, CV_MAKETYPE(CV_32F, MBParamsHOG::hogNumOrientations)),
        m_HOGFeatures(MBParamsHOG::hogNumRFY, MBParamsHOG::hogNumRFX, CV_MAKETYPE(CV_32F, MBParamsHOG::hogNumOrientations)),
        m_NumPNSpikes(0), m_NumKCSpikes(0), m_NumENSpikes(0), m_NumUsedWeights(MBParamsHOG::numKC * MBParamsHOG::numEN),
        m_NumActivePN(0), m_NumActiveKC(0), m_PNToKCTauSyn(3.0f), m_PNTauM(10.0), m_PNC(1.0),
        m_KCToENDopamineStrength(MBParamsHOG::dopamineStrength), m_RewardTimeMs(MBParamsHOG::rewardTimeMs),
        m_PresentDurationMs(MBParamsHOG::presentDurationMs), m_KCToENSynape(893), m_SLM("", "mb_memory_hog")
{
    std::cout << "HOG feature vector length:" << MBParamsHOG::hogFeatureSize << std::endl;

    // Build orientation vectors
    radian_t orient = 90_deg;
    for(auto &d : m_HOGDirections) {
        d[0] = sin(orient);
        d[1] = cos(orient);
        orient += 60_deg;
    }


    std::mt19937 gen;

    {
        Timer<> timer("Allocation:");
        m_SLM.allocateMem();
    }

    {
        Timer<> timer("Initialization:");
        m_SLM.initialize();
    }

    {
        Timer<> timer("Building connectivity:");

        unsigned int *rowLengthpnToKC = m_SLM.getArray<unsigned int>("rowLengthpnToKC");
        unsigned int *indpnToKC = m_SLM.getArray<unsigned int>("indpnToKC");
        unsigned int *maxRowLengthpnToKC = m_SLM.getScalar<unsigned int>("maxRowLengthpnToKC");
        float *gkcToEN = m_SLM.getArray<float>("gkcToEN");
        GeNNUtils::buildFixedNumberPreConnector(MBParamsHOG::numPN, MBParamsHOG::numKC, MBParamsHOG::numPNSynapsesPerKC,
                                                rowLengthpnToKC, indpnToKC, *maxRowLengthpnToKC, gen);

        // Manually initialise weights
        // **NOTE** this is a little bit of a hack as we're only doing this so repeated calls to initialise won't overwrite
        std::fill_n(&gkcToEN[0], MBParamsHOG::numKC * MBParamsHOG::numEN, MBParamsHOG::kcToENWeight);
    }

    // Final setup
    {
        Timer<> timer("Sparse init:");
        m_SLM.initializeSparse();
    }

    // Get pointers to EGPs
    m_GGGNToKC = m_SLM.getScalar<float>("gggnToKC");
    m_GKCToGGN = m_SLM.getScalar<float>("gkcToGGN");
    m_GPNToKC = m_SLM.getScalar<float>("gpnToKC");
    m_VMidGGNToKC = m_SLM.getScalar<float>("VmidggnToKC");
    m_VSlopeGGNToKC = m_SLM.getScalar<float>("VslopeggnToKC");
    m_VThreshGGNToKC = m_SLM.getScalar<float>("VthreshggnToKC");
    m_IExtScalePN = m_SLM.getScalar<float>("IextScalePN");
    m_VThreshPN = m_SLM.getScalar<float>("VthreshPN");
    m_InjectDopamineKCToEN = m_SLM.getScalar<bool>("injectDopaminekcToEN");
    m_TDKCToEN = m_SLM.getScalar<float>("tDkcToEN");
    m_DKCToEN = m_SLM.getScalar<float>("dkcToEN");
    m_ExpTCPN = m_SLM.getScalar<float>("ExpTCPN");
    m_RmembranePN = m_SLM.getScalar<float>("RmembranePN");
    m_ExpDecaypnToKC = m_SLM.getScalar<float>("expDecaypnToKC");
    m_InitPNToKC = m_SLM.getScalar<float>("initpnToKC");


    // Get pointers to state variables
    m_IExtPN = m_SLM.getArray<float>("IextPN");
    m_TCKCToEN = m_SLM.getArray<float>("tCkcToEN");
    m_SpkCntEN = m_SLM.getArray<unsigned int>("glbSpkCntEN");
    m_SpkEN = m_SLM.getArray<unsigned int>("glbSpkEN");
    m_SpkCntKC = m_SLM.getArray<unsigned int>("glbSpkCntKC");
    m_SpkKC = m_SLM.getArray<unsigned int>("glbSpkKC");
    m_SpkCntPN = m_SLM.getArray<unsigned int>("glbSpkCntPN");
    m_SpkPN = m_SLM.getArray<unsigned int>("glbSpkPN");
    m_VGGN = m_SLM.getArray<float>("VGGN");
    m_InSynGGNToKC = m_SLM.getArray<float>("inSynggnToKC");
    m_CKCToEN = m_SLM.getArray<float>("ckcToEN");
    m_GKCToEN = m_SLM.getArray<float>("gkcToEN");
    m_VEN = m_SLM.getArray<float>("VEN");

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
MBMemoryHOG::~MBMemoryHOG()
{
}
//----------------------------------------------------------------------------
void MBMemoryHOG::train(const cv::Mat &image)
{
    present(image, true);
}
//----------------------------------------------------------------------------
float MBMemoryHOG::test(const cv::Mat &image) const
{
    // Get number of EN spikes
    return (float)std::get<2>(present(image, false));
}
//----------------------------------------------------------------------------
void MBMemoryHOG::clearMemory()
{
    throw std::runtime_error("MBMemory does not currently support clearing");
}
//----------------------------------------------------------------------------
void MBMemoryHOG::write(cv::FileStorage& fs) const
{
    fs << "config" << "{";
    fs << "rewardTimeMs" << m_RewardTimeMs;
    fs << "presentDurationMs" << m_PresentDurationMs;

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

    fs << "kcToEN" << "{";
    fs << "dopamineStrength" << m_KCToENDopamineStrength;
    fs << "}";
    fs << "}";
}
//----------------------------------------------------------------------------
void MBMemoryHOG::read(const cv::FileNode &node)
{
    cv::read(node["rewardTimeMs"], m_RewardTimeMs, m_RewardTimeMs);
    cv::read(node["presentDurationMs"], m_PresentDurationMs, m_PresentDurationMs);

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
        cv::read(kcToGGC["weight"], *m_GKCToEN, *m_GKCToEN);
    }

    const auto &pnToKC = node["pnToKC"];
    if(pnToKC.isMap()) {
        cv::read(pnToKC["weight"], *m_GPNToKC, *m_GPNToKC);
        cv::read(pnToKC["tauSyn"], m_PNToKCTauSyn, m_PNToKCTauSyn);
    }

    const auto &kcToEN = node["kcToEN"];
    if(kcToEN.isMap()) {
        cv::read(kcToEN["dopamineStrength"], m_KCToENDopamineStrength, m_KCToENDopamineStrength);
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
std::tuple<unsigned int, unsigned int, unsigned int> MBMemoryHOG::present(const cv::Mat &image, bool train) const
{
    BOB_ASSERT(image.cols == MBParamsHOG::inputWidth);
    BOB_ASSERT(image.rows == MBParamsHOG::inputHeight);
    BOB_ASSERT(image.type() == CV_8UC1);

    // Convert image to float
    image.convertTo(m_SnapshotFloat, CV_32FC1, 1.0 / 255.0);

    // Apply Sobel operator to image
    cv::Sobel(m_SnapshotFloat, m_SobelX, CV_32F, 1, 0, 1);
    cv::Sobel(m_SnapshotFloat, m_SobelY, CV_32F, 0, 1, 1);

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

    // Make sure KC state and GGN insyn are reset before simulation
    m_SLM.initialize();

    // Copy HOG features into external input current
    BOB_ASSERT(m_HOGFeatures.isContinuous());
    std::copy_n(reinterpret_cast<float*>(m_HOGFeatures.data), MBParamsHOG::hogFeatureSize, m_IExtPN);
    m_SLM.pushVarToDevice("PN", "Iext");

    // Reset model time
    m_SLM.setTimestep(0);
    m_SLM.setTime(0.0f);

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = convertMsToTimesteps(m_RewardTimeMs);
    const unsigned int endPresentTimestep = convertMsToTimesteps(m_PresentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(MBParamsHOG::postStimuliDurationMs);

    const long long duration = endPresentTimestep + postStimuliDuration;

    // Clear GGN voltage and reserve
    m_GGNVoltageHistory.clear();
    m_GGNVoltageHistory.reserve(duration);

    m_KCInhInSynHistory.clear();
    m_KCInhInSynHistory.reserve(duration);

    m_DKCToENHistory.clear();
    m_DKCToENHistory.reserve(duration);

    m_CKCToENHistory.clear();
    m_CKCToENHistory.reserve(duration);

    m_GKCToENHistory.clear();
    m_GKCToENHistory.reserve(duration);

    m_ENVoltageHistory.clear();
    m_ENVoltageHistory.reserve(duration);

    // Clear spike records
    m_PNSpikes.clear();
    m_KCSpikes.clear();
    m_ENSpikes.clear();

    std::bitset<MBParamsHOG::numPN> pnSpikeBitset;
    std::bitset<MBParamsHOG::numKC> kcSpikeBitset;

    // Reset time of last dopamine spike
    *m_TDKCToEN = std::numeric_limits<float>::lowest();

    // Loop through timesteps
    m_NumPNSpikes = 0;
    m_NumKCSpikes = 0;
    m_NumENSpikes = 0;
    while(m_SLM.getTimestep() < duration) {
        // If we should stop presenting image
        if(m_SLM.getTimestep() == endPresentTimestep) {
            std::fill_n(m_IExtPN, MBParamsHOG::numPN, 0.0f);

            // Copy external input current to device
            m_SLM.pushVarToDevice("PN", "Iext");
        }

        // If we should reward in this timestep, inject dopamine
        if(train && m_SLM.getTimestep() == rewardTimestep) {
            *m_InjectDopamineKCToEN = true;
        }

        // Simulate on GPU
        m_SLM.stepTime();

        // Download spikes
        m_SLM.pullCurrentSpikesFromDevice("PN");
        m_SLM.pullCurrentSpikesFromDevice("KC");
        m_SLM.pullCurrentSpikesFromDevice("EN");
        m_SLM.pullStateFromDevice("GGN");
        m_SLM.pullStateFromDevice("EN");

        // **NOTE** very sub-optimal as we only use first!
        m_SLM.pullStateFromDevice("ggnToKC");
        m_SLM.pullStateFromDevice("kcToEN");

        // If a dopamine spike has been injected this timestep
        if(*m_InjectDopamineKCToEN) {
            // Decay global dopamine traces
            *m_DKCToEN *= std::exp(-(m_SLM.getTime() - *m_TDKCToEN) / MBParamsHOG::tauD);

            // Add effect of dopamine spike
            *m_DKCToEN += m_KCToENDopamineStrength;

            // Update last reward time
            *m_TDKCToEN = m_SLM.getTime();

            // Clear dopamine injection flags
            *m_InjectDopamineKCToEN = false;
        }

        m_NumENSpikes += m_SpkCntEN[0];
        m_NumPNSpikes += m_SpkCntPN[0];
        m_NumKCSpikes += m_SpkCntKC[0];
        for(unsigned int i = 0; i < m_SpkCntPN[0]; i++) {
            pnSpikeBitset.set(m_SpkPN[i]);
        }

        for(unsigned int i = 0; i < m_SpkCntKC[0]; i++) {
            kcSpikeBitset.set(m_SpkKC[i]);
        }

        // Record spikes
        record(m_SLM.getTime(), m_SpkCntPN[0], m_SpkPN, m_PNSpikes);
        record(m_SLM.getTime(), m_SpkCntKC[0], m_SpkKC, m_KCSpikes);
        record(m_SLM.getTime(), m_SpkCntEN[0], m_SpkEN, m_ENSpikes);

        // Record GGN voltage
        m_GGNVoltageHistory.push_back(m_VGGN[0]);
        m_KCInhInSynHistory.push_back(m_InSynGGNToKC[0]);
        m_DKCToENHistory.push_back(*m_DKCToEN * std::exp(-(m_SLM.getTime() - *m_TDKCToEN) / MBParamsHOG::tauD));
        m_CKCToENHistory.push_back(m_CKCToEN[m_KCToENSynape]);
        m_GKCToENHistory.push_back(m_GKCToEN[m_KCToENSynape]);
        m_ENVoltageHistory.push_back(m_VEN[0]);
    }

#ifdef RECORD_TERMINAL_SYNAPSE_STATE
    // Download synaptic state
    m_SLM.pullStateFromDevice("EN");

    std::ofstream terminalStream("terminal_synaptic_state.csv");
    terminalStream << "Weight, Eligibility" << std::endl;
    for(unsigned int s = 0; s < MBParamsHOG::numKC * MBParamsHOG::numEN; s++) {
        terminalStream << m_GKCToEN[s] << "," << m_CKCToEN[s] * std::exp(-(t - m_TCKCToEN[s]) / 40.0) << std::endl;
    }
    std::cout << "Final dopamine level:" << *m_DKCToEN * std::exp(-(t - *m_TDKCToEN) / MBParamsHOG::tauD) << std::endl;
#endif  // RECORD_TERMINAL_SYNAPSE_STATE

    // Cache number of unique active cells
    m_NumActivePN = pnSpikeBitset.count();
    m_NumActiveKC = kcSpikeBitset.count();

    if(train) {
        constexpr unsigned int numWeights = MBParamsHOG::numKC * MBParamsHOG::numEN;

        // Pull weights from device
        m_SLM.pullVarFromDevice("kcToEN", "g");

        // Cache number of unused weights
        m_NumUsedWeights = numWeights - std::count(&m_GKCToEN[0], &m_GKCToEN[numWeights], 0.0f);
    }

    return std::make_tuple(m_NumPNSpikes, m_NumKCSpikes, m_NumENSpikes);
}
