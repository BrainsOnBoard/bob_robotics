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
MBMemory::MBMemory()
    :   Navigation::VisualNavigationBase(cv::Size(MBParams::inputWidth, MBParams::inputHeight))
#ifndef CPU_ONLY
        , m_SnapshotFloatGPU(MBParams::inputHeight, MBParams::inputWidth, CV_32FC1)
#endif  // CPU_ONLY
{

    std::mt19937 gen;

    {
        Timer<> timer("Allocation:");
        allocateMem();
    }

    {
        Timer<> timer("Initialization:");
        initialize();

        // Null unused external input pointers
        IextKC = nullptr;
        IextEN = nullptr;
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

    // Largest difference would be expressed by EN firing every timestep
    return (float)numENSpikes / (float)(convertMsToTimesteps(MBParams::presentDurationMs) + convertMsToTimesteps(MBParams::postStimuliDurationMs));
}
//----------------------------------------------------------------------------
std::tuple<unsigned int, unsigned int, unsigned int> MBMemory::present(const cv::Mat &image, bool train) const
{
    BOB_ASSERT(image.cols == MBParams::inputWidth);
    BOB_ASSERT(image.rows == MBParams::inputHeight);
    BOB_ASSERT(image.type() == CV_32FC1);

    Timer<> timer("\tSimulation:");

#ifndef CPU_ONLY
    // Upload final snapshot to GPU
    m_SnapshotFloatGPU.upload(image);

    // Extract device pointers and step
    auto snapshotPtrStep = (cv::cuda::PtrStep<float>)m_SnapshotFloatGPU;
    const unsigned int snapshotStep = snapshotPtrStep.step / sizeof(float);
    float *snapshotData = snapshotPtrStep.data;
#else
    // Extract step and data pointer directly from CPU Mat
    const unsigned int snapshotStep = image.cols;
    float *snapshotData = reinterpret_cast<float*>(image.data);
#endif

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = iT + convertMsToTimesteps(MBParams::rewardTimeMs);
    const unsigned int presentDuration = convertMsToTimesteps(MBParams::presentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(MBParams::postStimuliDurationMs);

    const unsigned int duration = presentDuration + postStimuliDuration;
    const unsigned long long endPresentTimestep = iT + presentDuration;
    const unsigned long long endTimestep = iT + duration;

    // Open CSV output files
#ifdef RECORD_SPIKES
    const float startTimeMs = t;
    GeNNUtils::SpikeCSVRecorder pnSpikes("pn_spikes.csv", glbSpkCntPN, glbSpkPN);
    GeNNUtils::SpikeCSVRecorder kcSpikes("kc_spikes.csv", glbSpkCntKC, glbSpkKC);
    GeNNUtils::SpikeCSVRecorder enSpikes("en_spikes.csv", glbSpkCntEN, glbSpkEN);

    std::bitset<MBParams::numPN> pnSpikeBitset;
    std::bitset<MBParams::numKC> kcSpikeBitset;
#endif  // RECORD_SPIKES

    // Update input data step
    IextStepPN = snapshotStep;

    // Loop through timesteps
    unsigned int numPNSpikes = 0;
    unsigned int numKCSpikes = 0;
    unsigned int numENSpikes = 0;
    while(iT < endTimestep)
    {
        // If we should be presenting an image
        if(iT < endPresentTimestep) {
            IextPN = snapshotData;
        }
        // Otherwise update offset to point to block of zeros
        else {
            IextPN = nullptr;
        }

        // If we should reward in this timestep, inject dopamine
        if(train && iT == rewardTimestep) {
            injectDopaminekcToEN = true;
        }

#ifndef CPU_ONLY
        // Simulate on GPU
        stepTimeGPU();

        // Download spikes
#ifdef RECORD_SPIKES
        pullPNCurrentSpikesFromDevice();
        pullKCCurrentSpikesFromDevice();
        pullENCurrentSpikesFromDevice();
#else
        CHECK_CUDA_ERRORS(cudaMemcpy(glbSpkCntPN, d_glbSpkCntPN, sizeof(unsigned int), cudaMemcpyDeviceToHost));
        CHECK_CUDA_ERRORS(cudaMemcpy(glbSpkCntKC, d_glbSpkCntKC, sizeof(unsigned int), cudaMemcpyDeviceToHost));
        CHECK_CUDA_ERRORS(cudaMemcpy(glbSpkCntEN, d_glbSpkCntEN, sizeof(unsigned int), cudaMemcpyDeviceToHost));
#endif
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

        numPNSpikes += spikeCount_PN;
        numKCSpikes += spikeCount_KC;
        numENSpikes += spikeCount_EN;
#ifdef RECORD_SPIKES
        for(unsigned int i = 0; i < spikeCount_PN; i++) {
            pnSpikeBitset.set(spike_PN[i]);
        }

        for(unsigned int i = 0; i < spikeCount_KC; i++) {
            kcSpikeBitset.set(spike_KC[i]);
        }
        // Record spikes
        pnSpikes.record(t - startTimeMs);
        kcSpikes.record(t - startTimeMs);
        enSpikes.record(t - startTimeMs);
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
    std::cout << "Final dopamine level:" << dkcToEN * std::exp(-(t - tDkcToEN) / MBParams::tauD) << std::endl;
#endif  // RECORD_TERMINAL_SYNAPSE_STATE

#ifdef RECORD_SPIKES
    std::ofstream activeNeuronStream("active_neurons.csv", std::ios_base::app);
    activeNeuronStream << pnSpikeBitset.count() << "," << kcSpikeBitset.count() << "," << numENSpikes << std::endl;
#endif  // RECORD_SPIKES
    if(train) {
        constexpr unsigned int numWeights = MBParams::numKC * MBParams::numEN;

#ifndef CPU_ONLY
        CHECK_CUDA_ERRORS(cudaMemcpy(gkcToEN, d_gkcToEN, numWeights * sizeof(scalar), cudaMemcpyDeviceToHost));
#endif  // CPU_ONLY

        unsigned int numUsedWeights = std::count(&gkcToEN[0], &gkcToEN[numWeights], 0.0f);
        std::cout << "\t" << numWeights - numUsedWeights << " unused weights" << std::endl;
    }

    return std::make_tuple(numPNSpikes, numKCSpikes, numENSpikes);
}
