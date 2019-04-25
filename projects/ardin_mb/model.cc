// GeNN includes
#include "modelSpec.h"

// BoB robotics includes
#include "genn_models/exp_curr.h"
#include "genn_models/lif.h"
#include "genn_models/stdp_dopamine.h"
#include "genn_utils/connectors.h"

// Model includes
#include "mb_params.h"

using namespace BoBRobotics;

//---------------------------------------------------------------------------
// Standard LIF model extended to take an additional
// input current from an extra global variable
//---------------------------------------------------------------------------
class LIFExtCurrent : public NeuronModels::Base
{
public:
    DECLARE_MODEL(LIFExtCurrent, 8, 2);

    SET_SIM_CODE(
        "if ($(RefracTime) <= 0.0)\n"
        "{\n"
        "   scalar Iext = 0.0f;\n"
        "   if($(Iext) != NULL) {\n"
        "       unsigned int x = $(id) % (unsigned int)($(Width));\n"
        "       unsigned int y = $(id) / (unsigned int)($(Width));\n"
        "       const unsigned int index = (y * $(IextStep)) + x;\n"
        "       Iext = $(IextScale) * $(Iext)[index];\n"
        "   }\n"
        "   scalar alpha = (($(Isyn) + Iext) * $(Rmembrane)) + $(Vrest);\n"
        "   $(V) = alpha - ($(ExpTC) * (alpha - $(V)));\n"
        "}\n"
        "else\n"
        "{\n"
        "  $(RefracTime) -= DT;\n"
        "}\n"
    );

    SET_THRESHOLD_CONDITION_CODE("$(RefracTime) <= 0.0 && $(V) >= $(Vthresh)");

    SET_RESET_CODE(
        "$(V) = $(Vreset);\n"
        "$(RefracTime) = $(TauRefrac);\n");

    SET_PARAM_NAMES({
        "C",            // 0 -Membrane capacitance
        "TauM",         // 1 - Membrane time constant [ms]
        "Vrest",        // 2 - Resting membrane potential [mV]
        "Vreset",       // 3 - Reset voltage [mV]
        "Vthresh",      // 4 - Spiking threshold [mV]
        "TauRefrac",    // 5 - Refractory time [ms]
        "IextScale",    // 7 - Scaling factor to apply to external current
        "Width",        // 8 - Width of population (pixels)
    });

    SET_DERIVED_PARAMS({
        {"ExpTC", [](const vector<double> &pars, double dt){ return std::exp(-dt / pars[1]); }},
        {"Rmembrane", [](const vector<double> &pars, double){ return  pars[1] / pars[0]; }}});

    SET_VARS({{"V", "scalar"}, {"RefracTime", "scalar"}});

    SET_EXTRA_GLOBAL_PARAMS({{"Iext", "float*"}, {"IextStep", "unsigned int"}});
};
IMPLEMENT_MODEL(LIFExtCurrent);

void modelDefinition(NNmodel &model)
{
    GENN_PREFERENCES::autoInitSparseVars = true;
    GENN_PREFERENCES::defaultVarMode = VarMode::LOC_DEVICE_INIT_DEVICE;

    initGeNN();
    model.setDT(MBParams::timestepMs);
    model.setName("ardin_mb");

    //---------------------------------------------------------------------------
    // Neuron model parameters
    //---------------------------------------------------------------------------
    // LIF model parameters
    LIFExtCurrent::ParamValues pnParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        -50.0,                              // 4 - Vthresh
        2.0,                                // 5 - TauRefrac
        MBParams::inputCurrentScale,      // 6 - Scaling factor to apply to external current
        MBParams::inputWidth);            // 7 - Input width

    GeNNModels::LIF::ParamValues kcParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        -50.0,                              // 4 - Vthresh
        0.0,                                // 5 - Ioffset
        2.0);                               // 6 - TauRefrac

    GeNNModels::LIF::ParamValues enParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        -50.0,                              // 4 - Vthresh
        0.0,                                // 5 - Ioffset
        2.0);                               // 6 - TauRefrac

    // LIF initial conditions
    GeNNModels::LIF::VarValues lifInit(
        -60.0,  // 0 - V
        0.0);   // 1 - RefracTime

    //---------------------------------------------------------------------------
    // Postsynaptic model parameters
    //---------------------------------------------------------------------------
    GeNNModels::ExpCurr::ParamValues pnToKCPostsynapticParams(
        3.0);   // 0 - Synaptic time constant (ms)

    GeNNModels::ExpCurr::ParamValues kcToENPostsynapticParams(
        8.0);   // 0 - Synaptic time constant (ms)

    //---------------------------------------------------------------------------
    // Weight update model parameters
    //---------------------------------------------------------------------------
    WeightUpdateModels::StaticPulse::VarValues pnToKCWeightUpdateParams(MBParams::pnToKCWeight);

    GeNNModels::STDPDopamine::ParamValues kcToENWeightUpdateParams(
        15.0,                       // 0 - Potentiation time constant (ms)
        15.0,                       // 1 - Depression time constant (ms)
        40.0,                       // 2 - Synaptic tag time constant (ms)
        MBParams::tauD,           // 3 - Dopamine time constant (ms)
        -1.0,                       // 4 - Rate of potentiation
        1.0,                        // 5 - Rate of depression
        0.0,                        // 6 - Minimum weight
        MBParams::kcToENWeight);  // 7 - Maximum weight

    GeNNModels::STDPDopamine::VarValues kcToENWeightUpdateInitVars(
        MBParams::kcToENWeight,   // Synaptic weight
        0.0,                        // Synaptic tag
        0.0);                       // Time of last synaptic tag update

    // Create neuron populations
    auto pn = model.addNeuronPopulation<LIFExtCurrent>("PN", MBParams::numPN, pnParams, lifInit);
    auto kc = model.addNeuronPopulation<GeNNModels::LIF>("KC", MBParams::numKC, kcParams, lifInit);
    auto en = model.addNeuronPopulation<GeNNModels::LIF>("EN", MBParams::numEN, enParams, lifInit);
    pn->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);
    kc->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);
    en->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);

    auto pnToKC = model.addSynapsePopulation<WeightUpdateModels::StaticPulse, GeNNModels::ExpCurr>(
        "pnToKC", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "PN", "KC",
        {}, pnToKCWeightUpdateParams,
        pnToKCPostsynapticParams, {});

    auto kcToEN = model.addSynapsePopulation<GeNNModels::STDPDopamine, GeNNModels::ExpCurr>(
        "kcToEN", SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "KC", "EN",
        kcToENWeightUpdateParams, kcToENWeightUpdateInitVars,
        kcToENPostsynapticParams, {});


    // Calculate max connections
    const unsigned int maxConn = GeNNUtils::calcFixedNumberPreConnectorMaxConnections(MBParams::numPN, MBParams::numKC,
                                                                                      MBParams::numPNSynapsesPerKC);

    std::cout << "Max connections:" << maxConn << std::endl;
    pnToKC->setMaxConnections(maxConn);
    kcToEN->setWUVarMode("g", VarMode::LOC_HOST_DEVICE_INIT_DEVICE);

    model.finalize();
}
