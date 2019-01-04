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
        m_HOGFeatures(MBParams::hogFeatureSize), m_NumUsedWeights(MBParams::numKC * MBParams::numEN),
        m_NumActivePN(0), m_NumActiveKC(0)
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

        GeNNUtils::buildFixedNumberPreConnector(MBParams::numPN, MBParams::numKC,
                                                MBParams::numPNSynapsesPerKC, CpnToKC, &allocatepnToKC, gen);
    }

    // Final setup
    {
        Timer<> timer("Sparse init:");
        initardin_mb();
    }
#ifndef CPU_ONLY
    CHECK_CUDA_ERRORS(cudaMalloc(&m_HOGFeaturesGPU, MBParams::hogFeatureSize * sizeof(float)));
#endif
}
//----------------------------------------------------------------------------
MBMemoryHOG::~MBMemoryHOG()
{
#ifndef CPU_ONLY
    CHECK_CUDA_ERRORS(cudaFree(m_HOGFeaturesGPU));
#endif
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
std::tuple<unsigned int, unsigned int, unsigned int> MBMemoryHOG::present(const cv::Mat &image, bool train) const
{
    BOB_ASSERT(image.cols == MBParams::inputWidth);
    BOB_ASSERT(image.rows == MBParams::inputHeight);
    BOB_ASSERT(image.type() == CV_8UC1);

    // Compute HOG features
    m_HOG.compute(image, m_HOGFeatures);
    BOB_ASSERT(m_HOGFeatures.size() == MBParams::hogFeatureSize);

    const float magnitude = std::accumulate(m_HOGFeatures.begin(), m_HOGFeatures.end(), 0.0f);
    /*std::transform(m_HOGFeatures.begin(), m_HOGFeatures.end(), m_HOGFeatures.begin(),
                   [magnitude](float f)
                   {
                       return f / magnitude;
                   });*/
    //std::cout << "HOG feature magnitude:" << magnitude << std::endl;

    // Copy HOG features into GeNN variable
    std::copy(m_HOGFeatures.begin(), m_HOGFeatures.end(), ratePN);
    std::fill_n(timeStepToSpikePN, MBParams::numPN, 0.0f);
#ifndef CPU_ONLY
    pushPNStateToDevice();
#endif

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = iT + convertMsToTimesteps(MBParams::rewardTimeMs);
    const unsigned int presentDuration = convertMsToTimesteps(MBParams::presentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(MBParams::postStimuliDurationMs);

    const unsigned int duration = presentDuration + postStimuliDuration;
    const unsigned long long endPresentTimestep = iT + presentDuration;
    const unsigned long long endTimestep = iT + duration;

    // Clear GGN voltage and reserve
    m_GGNVoltage.clear();
    m_GGNVoltage.reserve(duration);

    // Open CSV output files
    const float startTimeMs = t;

    // Clear spike records
    m_PNSpikes.clear();
    m_KCSpikes.clear();
    m_ENSpikes.clear();

    std::bitset<MBParams::numPN> pnSpikeBitset;
    std::bitset<MBParams::numKC> kcSpikeBitset;

    // Loop through timesteps
    unsigned int numPNSpikes = 0;
    unsigned int numKCSpikes = 0;
    unsigned int numENSpikes = 0;
    while(iT < endTimestep) {
        // If we should stop presenting image
        if(iT == endPresentTimestep) {
            std::fill_n(timeStepToSpikePN, MBParams::numPN, 1000000.0f);

#ifndef CPU_ONLY
            pushPNStateToDevice();
#endif
        }

        // If we should reward in this timestep, inject dopamine
        if(train && iT == rewardTimestep) {
            injectDopaminekcToEN = true;
        }

#ifndef CPU_ONLY
        // Simulate on GPU
        stepTimeGPU();

        // Download spikes
        pullPNCurrentSpikesFromDevice();
        pullKCCurrentSpikesFromDevice();
        pullENCurrentSpikesFromDevice();
        pullGGNStateFromDevice();
#else
        // Simulate on CPU
        stepTimeCPU();
#endif
        // If a dopamine spike has been injected this timestep
        if(injectDopaminekcToEN) {
            // Decay global dopamine traces
            dkcToEN = dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD);

            // Add effect of dopamine spike
            dkcToEN += MBParams::dopamineStrength;

            // Update last reward time
            tDkcToEN = t;

            // Clear dopamine injection flags
            injectDopaminekcToEN = false;
        }

        numENSpikes += spikeCount_EN;
        numPNSpikes += spikeCount_PN;
        numKCSpikes += spikeCount_KC;
        for(unsigned int i = 0; i < spikeCount_PN; i++) {
            pnSpikeBitset.set(spike_PN[i]);
        }

        for(unsigned int i = 0; i < spikeCount_KC; i++) {
            kcSpikeBitset.set(spike_KC[i]);
        }

        // Record spikes
        record(t - startTimeMs, spikeCount_PN, spike_PN, m_PNSpikes);
        record(t - startTimeMs, spikeCount_KC, spike_KC, m_KCSpikes);
        record(t - startTimeMs, spikeCount_EN, spike_EN, m_ENSpikes);

        // Record GGN voltage
        m_GGNVoltage.push_back(VGGN[0]);
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

#ifndef CPU_ONLY
        CHECK_CUDA_ERRORS(cudaMemcpy(gkcToEN, d_gkcToEN, numWeights * sizeof(scalar), cudaMemcpyDeviceToHost));
#endif  // CPU_ONLY

        // Cache number of unused weights
        m_NumUsedWeights = numWeights - std::count(&gkcToEN[0], &gkcToEN[numWeights], 0.0f);
    }

    return std::make_tuple(numPNSpikes, numKCSpikes, numENSpikes);
}
