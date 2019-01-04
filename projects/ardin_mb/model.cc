// GeNN includes
#include "modelSpec.h"

// BoB robotics includes
#include "../../genn_models/exp_curr.h"
#include "../../genn_models/lif.h"
#include "../../genn_models/stdp_dopamine.h"
#include "../../genn_utils/connectors.h"

// Model includes
#include "mb_params.h"

using namespace BoBRobotics;

//---------------------------------------------------------------------------
// PoissonInput
//---------------------------------------------------------------------------
class PoissonInput : public NeuronModels::Base
{
public:
    DECLARE_MODEL(PoissonInput, 1, 2);

    SET_SIM_CODE(
        "if($(timeStepToSpike) <= 0.0f) {\n"
        "    $(timeStepToSpike) += (1000.0 / ($(rateScale) * $(rate) * DT)) * $(gennrand_exponential);\n"
        "}\n"
        "$(timeStepToSpike) -= 1.0;\n"
    );

    SET_THRESHOLD_CONDITION_CODE("$(timeStepToSpike) <= 0.0");

    SET_PARAM_NAMES({"rateScale"});
    SET_VARS({{"timeStepToSpike", "scalar"}, {"rate", "scalar"}});
};
IMPLEMENT_MODEL(PoissonInput);

//---------------------------------------------------------------------------
// ExpStaticGraded
//---------------------------------------------------------------------------
class ExpStaticGraded : public WeightUpdateModels::Base
{
public:
    DECLARE_WEIGHT_UPDATE_MODEL(ExpStaticGraded, 3, 0, 0, 0);

    SET_PARAM_NAMES({"Vmid", "Vslope", "Vthresh"});
    SET_EXTRA_GLOBAL_PARAMS({{"g", "scalar"}});

    SET_EVENT_CODE("$(addToInSyn, DT * $(g) * max(0.0, 1.0 / (1.0 + exp(($(Vmid) - $(V_pre)) / $(Vslope)))));\n");

    SET_EVENT_THRESHOLD_CONDITION_CODE("$(V_pre) > $(Vthresh)");
};
IMPLEMENT_MODEL(ExpStaticGraded);

//---------------------------------------------------------------------------
// StaticPulseEGP
//---------------------------------------------------------------------------
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
    GENN_PREFERENCES::autoInitSparseVars = true;
    GENN_PREFERENCES::defaultVarMode = VarMode::LOC_HOST_DEVICE_INIT_DEVICE;

    initGeNN();
    model.setDT(MBParams::timestepMs);
    model.setName("ardin_mb");

    //---------------------------------------------------------------------------
    // Neuron model parameters
    //---------------------------------------------------------------------------
    // LIF model parameters
    PoissonInput::ParamValues pnParams(
        MBParams::inputRateScale);       // 0 - rateScale

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
        1.0);                               // 6 - TauRefrac

    GeNNModels::LIF::ParamValues ggnParams(
        0.2,                                // 0 - C
        20.0,                               // 1 - TauM
        -60.0,                              // 2 - Vrest
        -60.0,                              // 3 - Vreset
        10000.0,                            // 4 - Vthresh
        0.0,                                // 5 - Ioffset
        2.0);                               // 6 - TauRefrac

    // LIF initial conditions
    GeNNModels::LIF::VarValues lifInit(
        -60.0,  // 0 - V
        0.0);   // 1 - RefracTime

    // Poisson input initial conditions
    PoissonInput::VarValues pnInit(
        0.0,                    // 0 - timeStepToSpike
        uninitialisedVar());    // 1 - rate

    //---------------------------------------------------------------------------
    // Postsynaptic model parameters
    //---------------------------------------------------------------------------
    GeNNModels::ExpCurr::ParamValues pnToKCPostsynapticParams(
        3.0);   // 0 - Synaptic time constant (ms)

    GeNNModels::ExpCurr::ParamValues kcToENPostsynapticParams(
        8.0);   // 0 - Synaptic time constant (ms)

    // **TODO** experiment with tuning these
    GeNNModels::ExpCurr::ParamValues kcToGGNPostsynapticParams(
        5.0);   // 0 - Synaptic time constant (ms)

    GeNNModels::ExpCurr::ParamValues ggnToKCPostsynapticParams(
        4.0);   // 0 - Synaptic time constant (ms)

    //---------------------------------------------------------------------------
    // Weight update model parameters
    //---------------------------------------------------------------------------
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

    ExpStaticGraded::ParamValues ggnToKCWeightUpdateParams(
        -40.0,  // 0 - Vmid
        2.0,    // 1 - Vslope
        -60.0); // 2 - Vthresh

    // Create neuron populations
    auto pn = model.addNeuronPopulation<PoissonInput>("PN", MBParams::numPN, pnParams, pnInit);
    auto kc = model.addNeuronPopulation<GeNNModels::LIF>("KC", MBParams::numKC, kcParams, lifInit);
    auto en = model.addNeuronPopulation<GeNNModels::LIF>("EN", MBParams::numEN, enParams, lifInit);
    model.addNeuronPopulation<GeNNModels::LIF>("GGN", 1, ggnParams, lifInit);
    pn->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);
    pn->setVarMode("rate", VarMode::LOC_HOST_DEVICE_INIT_HOST);
    pn->setVarMode("timeStepToSpike", VarMode::LOC_HOST_DEVICE_INIT_HOST);
    kc->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);
    en->setSpikeVarMode(VarMode::LOC_HOST_DEVICE_INIT_DEVICE);

    auto pnToKC = model.addSynapsePopulation<StaticPulseEGP, GeNNModels::ExpCurr>(
        "pnToKC", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "PN", "KC",
        {}, {},
        pnToKCPostsynapticParams, {});

    auto kcToEN = model.addSynapsePopulation<GeNNModels::STDPDopamine, GeNNModels::ExpCurr>(
        "kcToEN", SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "KC", "EN",
        kcToENWeightUpdateParams, kcToENWeightUpdateInitVars,
        kcToENPostsynapticParams, {});

    model.addSynapsePopulation<StaticPulseEGP, GeNNModels::ExpCurr>(
        "kcToGGN", SynapseMatrixType::DENSE_GLOBALG, NO_DELAY,
        "KC", "GGN",
        {}, {},
        kcToGGNPostsynapticParams, {});

    model.addSynapsePopulation<ExpStaticGraded, GeNNModels::ExpCurr>(
        "ggnToKC", SynapseMatrixType::DENSE_GLOBALG, NO_DELAY,
        "GGN","KC",
        ggnToKCWeightUpdateParams, {},
        ggnToKCPostsynapticParams, {});

    // Calculate max connections
    const unsigned int maxConn = GeNNUtils::calcFixedNumberPreConnectorMaxConnections(MBParams::numPN, MBParams::numKC,
                                                                                      MBParams::numPNSynapsesPerKC);

    std::cout << "Max connections:" << maxConn << std::endl;
    pnToKC->setMaxConnections(maxConn);
    kcToEN->setWUVarMode("g", VarMode::LOC_HOST_DEVICE_INIT_DEVICE);

    model.finalize();
}
