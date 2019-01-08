#pragma once

//------------------------------------------------------------------------
// MBParams
//------------------------------------------------------------------------
namespace MBParams
{
    constexpr double timestepMs = 1.0;

    constexpr unsigned int inputWidth = 36;
    constexpr unsigned int inputHeight = 9;

    // HOG feature configuration
    constexpr int hogCellSize = 3;
    constexpr int hogNumOrientations = 3;

    // Calculate hog feature size
    constexpr int hogFeatureSize = hogNumOrientations * (inputWidth / hogCellSize) * (inputHeight / hogCellSize);

    // Network dimensions
    constexpr unsigned int numPN = (unsigned int)hogFeatureSize;
    constexpr unsigned int numKC = 20000;
    constexpr unsigned int numEN = 1;

    // Regime parameters
    constexpr double rewardTimeMs = 40.0;
    constexpr double presentDurationMs = 40.0;
    constexpr double postStimuliDurationMs = 200.0;

    // Scale applied to convert image data to input currents for PNs
    constexpr double inputCurrentScale = 30000.0;

    // Weight of static synapses between PN and KC populations
    // **NOTE** manually tuend to get approximately 200/20000 KC firing sparsity
    //constexpr double pnToKCWeight = 0.0525;// 0.0705;
    constexpr double pnToKCWeight = 0.025;

    // Initial/maximum weight of plastic synapses between KC and EN populations
    // **NOTE** note manually tuned to get 15-20 spikes for a novel image
    constexpr double kcToENWeight = 0.2;

    // **NOTE** manually tuned to result in a maximum depolarization of 20mv (peak membrane voltage of -40mv)
    constexpr double kcToGGNWeight = 0.01;

    // **NOTE** manually tuned to normalize number of active KCs
    constexpr double ggnToKCWeight = -1.905;

    // Time constant of dopamine
    constexpr double tauD = 20.0;

    constexpr double ggnToKCVMid = -40.0;

    constexpr double ggnToKCVslope = 2.0;

    constexpr double ggnToKCVthresh = -50.0;

    constexpr double pnVthresh = -50.0;

    // Scale of each dopamine 'spike'
    // **NOTE** manually tuned for one-shot learning - also close to BA/phi
    constexpr double dopamineStrength = 0.03;

    // How many PN neurons are connected to each KC
    constexpr unsigned int numPNSynapsesPerKC = 10;



}