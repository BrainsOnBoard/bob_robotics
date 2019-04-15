#include "mb_memory_hog.h"

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <random>

// BoB robotics includes
#include "common/timer.h"
#include "genn_utils/connectors.h"

// GeNN generated code includes
#include "ardin_mb_CODE/definitions.h"

// Antworld includes
#include "mb_params.h"

using namespace BoBRobotics;

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
unsigned int convertMsToTimesteps(double ms)
{
    return (unsigned int)std::round(ms / MBParams::timestepMs);
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
    :   Navigation::VisualNavigationBase(cv::Size(MBParams::inputWidth, MBParams::inputHeight)),
        m_HOGFeatures(MBParams::hogFeatureSize), m_NumPNSpikes(0), m_NumKCSpikes(0), m_NumENSpikes(0),
        m_NumUsedWeights(MBParams::numKC * MBParams::numEN), m_NumActivePN(0), m_NumActiveKC(0),
        m_PNToKCTauSyn(3.0f), m_PNTauM(10.0), m_PNC(1.0), m_KCToENDopamineStrength(MBParams::dopamineStrength),
        m_RewardTimeMs(MBParams::rewardTimeMs), m_PresentDurationMs(MBParams::presentDurationMs), m_KCToENSynape(893)
{
    std::cout << "HOG feature vector length:" << MBParams::hogFeatureSize << std::endl;

    // Configure HOG features
    m_HOG.winSize = cv::Size(MBParams::inputWidth, MBParams::inputHeight);
    m_HOG.blockSize = m_HOG.winSize;
    m_HOG.blockStride = m_HOG.winSize;
    m_HOG.cellSize = cv::Size(MBParams::hogCellSize, MBParams::hogCellSize);
    m_HOG.nbins = MBParams::hogNumOrientations;

    std::mt19937 gen;

    {
        Timer<> timer("Allocation:");
        allocateMem();
    }

    {
        Timer<> timer("Initialization:");
        initialize();
    }

    {
        Timer<> timer("Building connectivity:");

        GeNNUtils::buildFixedNumberPreConnector(MBParams::numPN, MBParams::numKC, MBParams::numPNSynapsesPerKC,
                                                rowLengthpnToKC, indpnToKC, maxRowLengthpnToKC, gen);

        // Manually initialise weights
        // **NOTE** this is a little bit of a hack as we're only doing this so repeated calls to initialise won't overwrite
        std::fill_n(&gkcToEN[0], MBParams::numKC * MBParams::numEN, MBParams::kcToENWeight);
    }

    // Final setup
    {
        Timer<> timer("Sparse init:");
        initializeSparse();
    }

    // Set initial weights
    *getGGNToKCWeight() = MBParams::ggnToKCWeight;
    *getKCToGGNWeight() = MBParams::kcToGGNWeight;
    *getPNToKC() = MBParams::pnToKCWeight;

    *getPNInputCurrentScale() = MBParams::inputCurrentScale;
    *getPNVthresh() = MBParams::pnVthresh;

    *getGGNToKCVMid() = MBParams::ggnToKCVMid;
    *getGGNToKCVslope() = MBParams::ggnToKCVslope;
    *getGGNToKCVthresh() = MBParams::ggnToKCVthresh;
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
    unsigned int numENSpikes = std::get<2>(present(image, false));
    //std::cout << "\t" << numENSpikes << " EN spikes" << std::endl;

    // Largest difference would be expressed by EN firing every timestep
    return (float)numENSpikes / (float)(convertMsToTimesteps(MBParams::presentDurationMs) + convertMsToTimesteps(MBParams::postStimuliDurationMs));
}
//----------------------------------------------------------------------------
void MBMemoryHOG::clearMemory()
{
    throw std::runtime_error("MBMemory does not currently support clearing");
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getGGNToKCWeight()
{
    return &gggnToKC;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getKCToGGNWeight()
{
    return &gkcToGGN;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getPNToKC()
{
    return &gpnToKC;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getGGNToKCVMid()
{
    return &VmidggnToKC;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getGGNToKCVslope()
{
    return &VslopeggnToKC;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getGGNToKCVthresh()
{
    return &VthreshggnToKC;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getPNInputCurrentScale()
{
    return &IextScalePN;
}
//----------------------------------------------------------------------------
float *MBMemoryHOG::getPNVthresh()
{
    return &VthreshPN;
}
//----------------------------------------------------------------------------
void MBMemoryHOG::write(cv::FileStorage& fs) const
{
    fs << "config" << "{";
    fs << "rewardTimeMs" << m_RewardTimeMs;
    fs << "presentDurationMs" << m_PresentDurationMs;

    fs << "pn" << "{";
    fs << "inputCurrentScale" << IextScalePN;
    fs << "vThresh" << VthreshPN;
    fs << "cm" << m_PNC;
    fs << "tauM" << m_PNTauM;
    fs << "}";

    fs << "ggnToKC" << "{";
    fs << "weight" << gggnToKC;
    fs << "vMid" << VmidggnToKC;
    fs << "vSlope" << VslopeggnToKC;
    fs << "vThresh" << VthreshggnToKC;
    fs << "}";

    fs << "kcToGGC" << "{";
    fs << "weight" << gkcToGGN;
    fs << "}";

    fs << "pnToKC" << "{";
    fs << "weight" << gpnToKC;
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
        cv::read(pn["inputCurrentScale"], IextScalePN, IextScalePN);
        cv::read(pn["vThresh"], VthreshPN, VthreshPN);
        cv::read(pn["cm"], m_PNC, m_PNC);
        cv::read(pn["tauM"], m_PNTauM, m_PNTauM);
    }

    const auto &ggnToKC = node["ggnToKC"];
    if(ggnToKC.isMap()) {
        cv::read(ggnToKC["weight"], gggnToKC, gggnToKC);
        cv::read(ggnToKC["vMid"], VmidggnToKC, VmidggnToKC);
        cv::read(ggnToKC["vSlope"], VslopeggnToKC, VslopeggnToKC);
        cv::read(ggnToKC["vThresh"], VthreshggnToKC, VthreshggnToKC);
    }

    const auto &kcToGGC = node["kcToGGC"];
    if(kcToGGC.isMap()) {
        cv::read(kcToGGC["weight"], gkcToGGN, gkcToGGN);
    }

    const auto &pnToKC = node["pnToKC"];
    if(pnToKC.isMap()) {
        cv::read(pnToKC["weight"], gpnToKC, gpnToKC);
        cv::read(pnToKC["tauSyn"], m_PNToKCTauSyn, m_PNToKCTauSyn);
    }

    const auto &kcToEN = node["kcToEN"];
    if(kcToEN.isMap()) {
        cv::read(kcToEN["dopamineStrength"], m_KCToENDopamineStrength, m_KCToENDopamineStrength);
    }

}
//----------------------------------------------------------------------------
std::tuple<unsigned int, unsigned int, unsigned int> MBMemoryHOG::present(const cv::Mat &image, bool train) const
{
    BOB_ASSERT(image.cols == MBParams::inputWidth);
    BOB_ASSERT(image.rows == MBParams::inputHeight);
    BOB_ASSERT(image.type() == CV_8UC1);

    // Compute HOG features
    m_HOG.compute(image, m_HOGFeatures);
    BOB_ASSERT(m_HOGFeatures.size() == MBParams::hogFeatureSize);

    // Set extra global params
    ExpTCPN = std::exp(-MBParams::timestepMs / m_PNTauM);
    RmembranePN = m_PNTauM / m_PNC;
    expDecaypnToKC = (float)std::exp(-MBParams::timestepMs / m_PNToKCTauSyn);
    initpnToKC = (float)(m_PNToKCTauSyn * (1.0 - std::exp(-MBParams::timestepMs / m_PNToKCTauSyn))) * (1.0 / MBParams::timestepMs);

    // Make sure KC state and GGN insyn are reset before simulation
    initialize();

    // Copy HOG features into external input current
    std::copy(m_HOGFeatures.begin(), m_HOGFeatures.end(), IextPN);
    pushIextPNToDevice();

    // Reset model time
    iT = 0;
    t = 0.0f;

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = convertMsToTimesteps(m_RewardTimeMs);
    const unsigned int endPresentTimestep = convertMsToTimesteps(m_PresentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(MBParams::postStimuliDurationMs);

    const long long duration = endPresentTimestep + postStimuliDuration;

    // Clear GGN voltage and reserve
    m_GGNVoltage.clear();
    m_GGNVoltage.reserve(duration);

    m_KCInhInSyn.clear();
    m_KCInhInSyn.reserve(duration);

    m_DKCToEN.clear();
    m_DKCToEN.reserve(duration);

    m_CKCToEN.clear();
    m_CKCToEN.reserve(duration);

    m_GKCToEN.clear();
    m_GKCToEN.reserve(duration);

    m_ENVoltage.clear();
    m_ENVoltage.reserve(duration);

    // Open CSV output files
    const float startTimeMs = t;

    // Clear spike records
    m_PNSpikes.clear();
    m_KCSpikes.clear();
    m_ENSpikes.clear();

    std::bitset<MBParams::numPN> pnSpikeBitset;
    std::bitset<MBParams::numKC> kcSpikeBitset;

    // Reset time of last dopamine spike
    tDkcToEN = std::numeric_limits<float>::lowest();

    // Loop through timesteps
    m_NumPNSpikes = 0;
    m_NumKCSpikes = 0;
    m_NumENSpikes = 0;
    while(iT < duration) {
        // If we should stop presenting image
        if(iT == endPresentTimestep) {
            std::fill_n(IextPN, MBParams::numPN, 0.0f);

            // Copy external input current to device
            pushIextPNToDevice();
        }

        // If we should reward in this timestep, inject dopamine
        if(train && iT == rewardTimestep) {
            std::cout << "Injecting dopamine" << std::endl;
            injectDopaminekcToEN = true;
        }

        // Simulate on GPU
        stepTime();

        // Download spikes
        pullPNCurrentSpikesFromDevice();
        pullKCCurrentSpikesFromDevice();
        pullENCurrentSpikesFromDevice();
        pullGGNStateFromDevice();
        pullENStateFromDevice();

        // **NOTE** very sub-optimal as we only use first!
        pullggnToKCStateFromDevice();
        pullkcToENStateFromDevice();

        // If a dopamine spike has been injected this timestep
        if(injectDopaminekcToEN) {
            // Decay global dopamine traces
            dkcToEN = dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD);

            // Add effect of dopamine spike
            dkcToEN += m_KCToENDopamineStrength;

            // Update last reward time
            tDkcToEN = t;

            // Clear dopamine injection flags
            injectDopaminekcToEN = false;
        }

        m_NumENSpikes += spikeCount_EN;
        m_NumPNSpikes += spikeCount_PN;
        m_NumKCSpikes += spikeCount_KC;
        for(unsigned int i = 0; i < spikeCount_PN; i++) {
            pnSpikeBitset.set(spike_PN[i]);
        }

        for(unsigned int i = 0; i < spikeCount_KC; i++) {
            kcSpikeBitset.set(spike_KC[i]);
        }


        // Record spikes
        record(t, spikeCount_PN, spike_PN, m_PNSpikes);
        record(t, spikeCount_KC, spike_KC, m_KCSpikes);
        record(t, spikeCount_EN, spike_EN, m_ENSpikes);

        // Record GGN voltage
        m_GGNVoltage.push_back(VGGN[0]);
        m_KCInhInSyn.push_back(inSynggnToKC[0]);
        m_DKCToEN.push_back(dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD));
        m_CKCToEN.push_back(ckcToEN[m_KCToENSynape]);
        m_GKCToEN.push_back(gkcToEN[m_KCToENSynape]);
        m_ENVoltage.push_back(VEN[0]);
    }

#ifdef RECORD_TERMINAL_SYNAPSE_STATE
    // Download synaptic state
    pullkcToENStateFromDevice();

    std::ofstream terminalStream("terminal_synaptic_state.csv");
    terminalStream << "Weight, Eligibility" << std::endl;
    for(unsigned int s = 0; s < MBParams::numKC * MBParams::numEN; s++) {
        terminalStream << gkcToEN[s] << "," << ckcToEN[s] * std::exp(-(t - tCkcToEN[s]) / 40.0) << std::endl;
    }
    std::cout << "Final dopamine level:" << dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD) << std::endl;
#endif  // RECORD_TERMINAL_SYNAPSE_STATE

    // Cache number of unique active cells
    m_NumActivePN = pnSpikeBitset.count();
    m_NumActiveKC = kcSpikeBitset.count();

    if(train) {
        constexpr unsigned int numWeights = MBParams::numKC * MBParams::numEN;

        // Pull weights from device
        pullgkcToENFromDevice();

        // Cache number of unused weights
        m_NumUsedWeights = numWeights - std::count(&gkcToEN[0], &gkcToEN[numWeights], 0.0f);
    }

    return std::make_tuple(m_NumPNSpikes, m_NumKCSpikes, m_NumENSpikes);
}
