#include "mb_memory.h"

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <random>

// BoB robotics includes
#include "common/timer.h"
#include "genn_utils/connectors.h"
#include "genn_utils/spike_csv_recorder.h"

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
}   // Anonymous namespace

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
MBMemory::MBMemory(bool normaliseInput)
    :   Navigation::VisualNavigationBase(cv::Size(MBParams::inputWidth, MBParams::inputHeight)), m_NormaliseInput(normaliseInput),
        m_SnapshotFloat(MBParams::inputHeight, MBParams::inputWidth, CV_32FC1)
{

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
}
//----------------------------------------------------------------------------
void MBMemory::train(const cv::Mat &image)
{
    present(image, true);
}
//----------------------------------------------------------------------------
float MBMemory::test(const cv::Mat &image) const
{
    // Get number of EN spikes
    unsigned int numENSpikes = std::get<2>(present(image, false));
    //LOGI << "\t" << numENSpikes << " EN spikes";

    // Largest difference would be expressed by EN firing every timestep
    return (float)numENSpikes / (float)(convertMsToTimesteps(MBParams::presentDurationMs) + convertMsToTimesteps(MBParams::postStimuliDurationMs));
}
//----------------------------------------------------------------------------
void MBMemory::clearMemory()
{
    throw std::runtime_error("MBMemory does not currently support clearing");
}
//----------------------------------------------------------------------------
std::tuple<unsigned int, unsigned int, unsigned int> MBMemory::present(const cv::Mat &image, bool train) const
{
    BOB_ASSERT(image.cols == MBParams::inputWidth);
    BOB_ASSERT(image.rows == MBParams::inputHeight);
    BOB_ASSERT(image.type() == CV_8UC1);

    // Convert to float
    image.convertTo(m_SnapshotFloat, CV_32FC1, 1.0 / 255.0);

    // Normalise snapshot using L2 norm
    if(m_NormaliseInput) {
        cv::normalize(m_SnapshotFloat, m_SnapshotFloat);
    }

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = convertMsToTimesteps(MBParams::rewardTimeMs);
    const unsigned int endPresentTimestep = convertMsToTimesteps(MBParams::presentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(MBParams::postStimuliDurationMs);

    const unsigned long long duration = endPresentTimestep + postStimuliDuration;

    // Open CSV output files
#ifdef RECORD_SPIKES
    GeNNUtils::SpikeCSVRecorder pnSpikes("pn_spikes.csv", glbSpkCntPN, glbSpkPN);
    GeNNUtils::SpikeCSVRecorder kcSpikes("kc_spikes.csv", glbSpkCntKC, glbSpkKC);
    GeNNUtils::SpikeCSVRecorder enSpikes("en_spikes.csv", glbSpkCntEN, glbSpkEN);

    std::bitset<MBParams::numPN> pnSpikeBitset;
    std::bitset<MBParams::numKC> kcSpikeBitset;
#endif  // RECORD_SPIKES

    // Make sure KC state and GGN insyn are reset before simulation
    initialize();

    // Copy HOG features into external input current
    BOB_ASSERT(m_SnapshotFloat.isContinuous());
    std::copy_n(reinterpret_cast<float*>(m_SnapshotFloat.data), MBParams::numPN, IextPN);
    pushIextPNToDevice();

    // Reset model time
    iT = 0;
    t = 0.0f;

    // Loop through timesteps
    unsigned int numPNSpikes = 0;
    unsigned int numKCSpikes = 0;
    unsigned int numENSpikes = 0;
    while(iT < duration) {
        // If we should stop presenting image
        if(iT == endPresentTimestep) {
            std::fill_n(IextPN, MBParams::numPN, 0.0f);

            // Copy external input current to device
            pushIextPNToDevice();
        }

        // If we should reward in this timestep, inject dopamine
        if(train && iT == rewardTimestep) {
            injectDopaminekcToEN = true;
        }

        // Simulat
        stepTime();

        // Download spikes
#ifdef RECORD_SPIKES
        pullPNCurrentSpikesFromDevice();
        pullKCCurrentSpikesFromDevice();
        pullENCurrentSpikesFromDevice();
#else
        pullENCurrentSpikesFromDevice();
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
#ifdef RECORD_SPIKES
        numPNSpikes += spikeCount_PN;
        numKCSpikes += spikeCount_KC;
        for(unsigned int i = 0; i < spikeCount_PN; i++) {
            pnSpikeBitset.set(spike_PN[i]);
        }

        for(unsigned int i = 0; i < spikeCount_KC; i++) {
            kcSpikeBitset.set(spike_KC[i]);
        }
        // Record spikes
        pnSpikes.record(t);
        kcSpikes.record(t);
        enSpikes.record(t);
#endif  // RECORD_SPIKES
    }

#ifdef RECORD_TERMINAL_SYNAPSE_STATE
    // Download synaptic state
    pullkcToENStateFromDevice();

    std::ofstream terminalStream("terminal_synaptic_state.csv");
    terminalStream << "Weight, Eligibility" << std::endl;
    for(unsigned int s = 0; s < MBParams::numKC * MBParams::numEN; s++) {
        terminalStream << gkcToEN[s] << "," << ckcToEN[s] * std::exp(-(t - tCkcToEN[s]) / 40.0) << std::endl;
    }
    LOGI << "Final dopamine level:" << dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD);
#endif  // RECORD_TERMINAL_SYNAPSE_STATE

#ifdef RECORD_SPIKES
    std::ofstream activeNeuronStream("active_neurons.csv", std::ios_base::app);
    activeNeuronStream << pnSpikeBitset.count() << "," << kcSpikeBitset.count() << "," << numENSpikes << std::endl;
#endif  // RECORD_SPIKES
    if(train) {
        constexpr unsigned int numWeights = MBParams::numKC * MBParams::numEN;

        // Download weights
        pullgkcToENFromDevice();

        unsigned int numUsedWeights = std::count(&gkcToEN[0], &gkcToEN[numWeights], 0.0f);
        LOGI << "\t" << numWeights - numUsedWeights << " unused weights";
    }

    return std::make_tuple(numPNSpikes, numKCSpikes, numENSpikes);
}
