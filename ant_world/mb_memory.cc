#include "mb_memory.h"

// Standard C++ includes
#include <bitset>
#include <fstream>
#include <random>

// Common includes
#include "../common/connectors.h"
#include "../common/spike_csv_recorder.h"
#include "../common/timer.h"

// GeNN generated code includes
#include "ant_world_CODE/definitions.h"

// Antworld includes
#include "parameters.h"

using namespace GeNNRobotics;

//----------------------------------------------------------------------------
// Anonymous namespace
//----------------------------------------------------------------------------
namespace
{
unsigned int convertMsToTimesteps(double ms)
{
    return (unsigned int)std::round(ms / Parameters::timestepMs);
}
}   // Anonymous namespace

//----------------------------------------------------------------------------
// MBMemory
//----------------------------------------------------------------------------
MBMemory::MBMemory()
#ifndef CPU_ONLY
    : m_SnapshotFloatGPU(Parameters::inputWidth, Parameters::inputHeight, CV_32FC1)
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

        buildFixedNumberPreConnector(Parameters::numPN, Parameters::numKC,
                                     Parameters::numPNSynapsesPerKC, CpnToKC, &allocatepnToKC, gen);
    }

    // Final setup
    {
        Timer<> timer("Sparse init:");
        initant_world();
    }
}
//----------------------------------------------------------------------------
std::future<std::tuple<unsigned int, unsigned int, unsigned int>> MBMemory::present(const cv::Mat &snapshotFloat, bool train)
{
#ifndef CPU_ONLY
    // Upload final snapshot to GPU
    m_SnapshotFloatGPU.upload(snapshotFloat);

    // Extract device pointers and step
    auto snapshotPtrStep = (cv::cuda::PtrStep<float>)m_SnapshotFloatGPU;
    const unsigned int snapshotStep = snapshotPtrStep.step / sizeof(float);
    float *snapshotData = snapshotPtrStep.data;
#else
    // Extract step and data pointer directly from CPU Mat
    const unsigned int snapshotStep = snapshotFloat.cols;
    float *snapshotData = reinterpret_cast<float*>(snapshotFloat.data);
#endif

    // Start simulation, applying reward if we are training
    return std::async(std::launch::async, &MBMemory::presentThread,
                      this, snapshotData, snapshotStep, train);
}
//----------------------------------------------------------------------------
std::tuple<unsigned int, unsigned int, unsigned int> MBMemory::presentThread(float *inputData, unsigned int inputDataStep, bool reward)
{
    std::mt19937 gen;

    Timer<> timer("\tSimulation:");

    // Convert simulation regime parameters to timesteps
    const unsigned long long rewardTimestep = iT + convertMsToTimesteps(Parameters::rewardTimeMs);
    const unsigned int presentDuration = convertMsToTimesteps(Parameters::presentDurationMs);
    const unsigned int postStimuliDuration = convertMsToTimesteps(Parameters::postStimuliDurationMs);

    const unsigned int duration = presentDuration + postStimuliDuration;
    const unsigned long long endPresentTimestep = iT + presentDuration;
    const unsigned long long endTimestep = iT + duration;

    // Open CSV output files
#ifdef RECORD_SPIKES
    const float startTimeMs = t;
    SpikeCSVRecorder pnSpikes("pn_spikes.csv", glbSpkCntPN, glbSpkPN);
    SpikeCSVRecorder kcSpikes("kc_spikes.csv", glbSpkCntKC, glbSpkKC);
    SpikeCSVRecorder enSpikes("en_spikes.csv", glbSpkCntEN, glbSpkEN);

    std::bitset<Parameters::numPN> pnSpikeBitset;
    std::bitset<Parameters::numKC> kcSpikeBitset;
#endif  // RECORD_SPIKES

    // Update input data step
    IextStepPN = inputDataStep;

    // Loop through timesteps
    unsigned int numPNSpikes = 0;
    unsigned int numKCSpikes = 0;
    unsigned int numENSpikes = 0;
    while(iT < endTimestep)
    {
        // If we should be presenting an image
        if(iT < endPresentTimestep) {
            IextPN = inputData;
        }
        // Otherwise update offset to point to block of zeros
        else {
            IextPN = nullptr;
        }

        // If we should reward in this timestep, inject dopamine
        if(reward && iT == rewardTimestep) {
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
            dkcToEN = dkcToEN * std::exp(-(t - tDkcToEN) / Parameters::tauD);

            // Add effect of dopamine spike
            dkcToEN += Parameters::dopamineStrength;

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
    for(unsigned int s = 0; s < Parameters::numKC * Parameters::numEN; s++) {
        terminalStream << gkcToEN[s] << "," << ckcToEN[s] * std::exp(-(t - tCkcToEN[s]) / 40.0) << std::endl;
    }
    std::cout << "Final dopamine level:" << dkcToEN * std::exp(-(t - tDkcToEN) / Parameters::tauD) << std::endl;
#endif  // RECORD_TERMINAL_SYNAPSE_STATE

#ifdef RECORD_SPIKES
    std::ofstream activeNeuronStream("active_neurons.csv", std::ios_base::app);
    activeNeuronStream << pnSpikeBitset.count() << "," << kcSpikeBitset.count() << "," << numENSpikes << std::endl;
#endif  // RECORD_SPIKES
    if(reward) {
        constexpr unsigned int numWeights = Parameters::numKC * Parameters::numEN;

#ifndef CPU_ONLY
        CHECK_CUDA_ERRORS(cudaMemcpy(gkcToEN, d_gkcToEN, numWeights * sizeof(scalar), cudaMemcpyDeviceToHost));
#endif  // CPU_ONLY

        unsigned int numUsedWeights = std::count(&gkcToEN[0], &gkcToEN[numWeights], 0.0f);
        std::cout << "\t" << numWeights - numUsedWeights << " unused weights" << std::endl;
    }

    return std::make_tuple(numPNSpikes, numKCSpikes, numENSpikes);
}
