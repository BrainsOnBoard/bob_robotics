// GeNN includes
#include "modelSpec.h"

// BoB robotics includes
#include "../../include/genn_models/stdp_dopamine.h"
#include "../../include/genn_utils/connectors.h"

// Model includes
#include "mb_params_hog.h"

using namespace BoBRobotics;


//---------------------------------------------------------------------------
// Standard LIF model extended to take an additional
// input current from an extra global variable
//---------------------------------------------------------------------------
class LIFExtCurrent : public NeuronModels::Base
{
public:
    DECLARE_MODEL(LIFExtCurrent, 3, 3);

    SET_SIM_CODE(
        "if ($(RefracTime) <= 0.0)\n"
        "{\n"
        "   scalar alpha = (($(Isyn) + ($(IextScale) * $(Iext))) * $(Rmembrane)) + $(Vrest);\n"
        "   $(V) = alpha - ($(ExpTC) * (alpha - $(V)));\n"
        "}\n"
        "else\n"
        "{\n"
        "  $(RefracTime) -= DT;\n"
        "}\n");

    SET_THRESHOLD_CONDITION_CODE("$(RefracTime) <= 0.0 && $(V) >= $(Vthresh)");

    SET_RESET_CODE(
        "$(V) = $(Vreset);\n"
        "$(RefracTime) = $(TauRefrac);\n");

    SET_PARAM_NAMES({
        "Vrest",        // 0 - Resting membrane potential [mV]
        "Vreset",       // 1 - Reset voltage [mV]
        "TauRefrac"});  // 2 - Scaling factor to apply to external current

    SET_VARS({{"V", "scalar"}, {"RefracTime", "scalar"}, {"Iext", "scalar"}});

    SET_EXTRA_GLOBAL_PARAMS({{"IextScale", "float"}, {"Vthresh", "scalar"}, {"ExpTC", "scalar"}, {"Rmembrane", "scalar"}});
};
IMPLEMENT_MODEL(LIFExtCurrent);

//---------------------------------------------------------------------------
// ExpCurrEGP
//---------------------------------------------------------------------------
// Standard exp curr P.S.M. modified so tau can be controlled with E.G.P.
class ExpCurrEGP : public PostsynapticModels::Base
{
public:
    DECLARE_MODEL(ExpCurrEGP, 0, 0);

    SET_DECAY_CODE("$(inSyn)*=$(expDecay);");

    SET_CURRENT_CONVERTER_CODE("$(init) * $(inSyn)");

    SET_EXTRA_GLOBAL_PARAMS({{"expDecay", "scalar"}, {"init", "scalar"}});
};
IMPLEMENT_MODEL(ExpCurrEGP);

//---------------------------------------------------------------------------
// ExpStaticGraded
//---------------------------------------------------------------------------
class ExpStaticGraded : public WeightUpdateModels::Base
{
public:
    DECLARE_WEIGHT_UPDATE_MODEL(ExpStaticGraded, 0, 0, 0, 0);

    SET_EXTRA_GLOBAL_PARAMS({{"g", "scalar"}, {"Vmid", "scalar"}, {"Vslope", "scalar"}, {"Vthresh", "scalar"}});

    SET_EVENT_CODE("$(addToInSyn, DT * $(g) * max(0.0, 1.0 / (1.0 + exp(($(Vmid) - $(V_pre)) / $(Vslope)))));\n");

    SET_EVENT_THRESHOLD_CONDITION_CODE("$(V_pre) > $(Vthresh)");
};
IMPLEMENT_MODEL(ExpStaticGraded);

//---------------------------------------------------------------------------
// StaticPulseEGP
//---------------------------------------------------------------------------
// Standard static pulse W.U.M. modified so weight can be controlled with E.G.P.
class StaticPulseEGP : public WeightUpdateModels::Base
{
public:
    DECLARE_WEIGHT_UPDATE_MODEL(StaticPulseEGP, 0, 0, 0, 0);

    SET_EXTRA_GLOBAL_PARAMS({{"g", "scalar"}});

    SET_SIM_CODE("$(addToInSyn, $(g));\n");
};
IMPLEMENT_MODEL(StaticPulseEGP);

void modelDefinition(NNmodel &model)
{
    model.setDT(MBParamsHOG::timestepMs);
    model.setName("mb_memory_hog");

    //---------------------------------------------------------------------------
    // Neuron model parameters
    //---------------------------------------------------------------------------
    // LIF model parameters
    LIFExtCurrent::ParamValues pnParams(
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        200.0);                               // 4 - TauRefrac **NOTE** essentially make neurons fire once

    NeuronModels::LIF::ParamValues kcParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        -50.0,                              // 4 - Vthresh
        0.0,                                // 5 - Ioffset
        2.0);                               // 6 - TauRefrac

    NeuronModels::LIF::ParamValues enParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        -50.0,                              // 4 - Vthresh
        0.0,                                // 5 - Ioffset
        1.0);                               // 6 - TauRefrac

    NeuronModels::LIF::ParamValues ggnParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        10000.0,                            // 4 - Vthresh **NOTE** essentially non-spiking
        0.0,                                // 5 - Ioffset
        2.0);                               // 6 - TauRefrac

    // LIF initial conditions
    NeuronModels::LIF::VarValues lifInit(
        -60.0,  // 0 - V
        0.0);   // 1 - RefracTime

    // PN initial conditions
    LIFExtCurrent::VarValues pnInit(
        -60.0,  // 0 - V
        0.0,    // 1 - RefracTime
        0.0);   // 2 - Iext

    //---------------------------------------------------------------------------
    // Postsynaptic model parameters
    //---------------------------------------------------------------------------
    PostsynapticModels::ExpCurr::ParamValues kcToENPostsynapticParams(
        8.0);   // 0 - Synaptic time constant [from Ardin] (ms)

    // **TODO** experiment with tuning these
    PostsynapticModels::ExpCurr::ParamValues kcToGGNPostsynapticParams(
        5.0);   // 0 - Synaptic time constant (ms)

    PostsynapticModels::ExpCurr::ParamValues ggnToKCPostsynapticParams(
        4.0);   // 0 - Synaptic time constant [from Nowotny](ms)

    //---------------------------------------------------------------------------
    // Weight update model parameters
    //---------------------------------------------------------------------------
    GeNNModels::STDPDopamine::ParamValues kcToENWeightUpdateParams(
        15.0,                       // 0 - Potentiation time constant (ms)
        15.0,                       // 1 - Depression time constant (ms)
        40.0,                       // 2 - Synaptic tag time constant (ms)
        MBParamsHOG::tauD,           // 3 - Dopamine time constant (ms)
        -1.0,                       // 4 - Rate of potentiation
        1.0,                        // 5 - Rate of depression
        0.0,                        // 6 - Minimum weight
        MBParamsHOG::kcToENWeight);  // 7 - Maximum weight

    GeNNModels::STDPDopamine::VarValues kcToENWeightUpdateInitVars(
        uninitialisedVar(),         // Synaptic weight
        0.0,                        // Synaptic tag
        0.0);                       // Time of last synaptic tag update

    // Create neuron populations
    model.addNeuronPopulation<LIFExtCurrent>("PN", MBParamsHOG::numPN, pnParams, pnInit);
    model.addNeuronPopulation<NeuronModels::LIF>("KC", MBParamsHOG::numKC, kcParams, lifInit);
    model.addNeuronPopulation<NeuronModels::LIF>("EN", MBParamsHOG::numEN, enParams, lifInit);
    model.addNeuronPopulation<NeuronModels::LIF>("GGN", 1, ggnParams, lifInit);

    auto pnToKC = model.addSynapsePopulation<StaticPulseEGP, ExpCurrEGP>(
        "pnToKC", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "PN", "KC",
        {}, {},
        {}, {});

    model.addSynapsePopulation<GeNNModels::STDPDopamine, PostsynapticModels::ExpCurr>(
        "kcToEN", SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "KC", "EN",
        kcToENWeightUpdateParams, kcToENWeightUpdateInitVars,
        kcToENPostsynapticParams, {});

    model.addSynapsePopulation<StaticPulseEGP, PostsynapticModels::ExpCurr>(
        "kcToGGN", SynapseMatrixType::DENSE_GLOBALG, NO_DELAY,
        "KC", "GGN",
        {}, {},
        kcToGGNPostsynapticParams, {});

    model.addSynapsePopulation<ExpStaticGraded, PostsynapticModels::ExpCurr>(
        "ggnToKC", SynapseMatrixType::DENSE_GLOBALG, NO_DELAY,
        "GGN","KC",
        {}, {},
        ggnToKCPostsynapticParams, {});

    // Calculate max connections
    const unsigned int maxConn = GeNNUtils::calcFixedNumberPreConnectorMaxConnections(MBParamsHOG::numPN, MBParamsHOG::numKC,
                                                                                      MBParamsHOG::numPNSynapsesPerKC);

    std::cout << "Max connections:" << maxConn << std::endl;
    pnToKC->setMaxConnections(maxConn);
}
