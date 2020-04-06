// 
#include "modelSpec.h"

class DE : public CurrentSourceModels::Base
{
    DECLARE_MODEL(DE, 0, 1);

    SET_INJECTION_CODE("$(injectCurrent, $(amp));\n");

    SET_VARS({{"amp", "scalar"}});
};
IMPLEMENT_MODEL(DE);


void modelDefinition(NNmodel &model) 
{
    model.setDT(1);
    model.setName("radar_bot");

    //---------------------------------------------------------------------------
    // Build model
    //---------------------------------------------------------------------------
 
    NeuronModels::LIF::ParamValues HD_params(
        0.25,               // 0 - C
        10.0,               // 1 - TauM
        -65.0,              // 2 - Vrest
        -65.0,              // 3 - Vreset
        -50.0,              // 4 - Vthresh
        0.0,                // 5 - Ioffset
        0.0               // 6 - TauRefrac
    );

    DE::VarValues stimVars(0.0);
    // init parameters
    NeuronModels::LIF::VarValues InitVals( 0.0, 0.0);
    PostsynapticModels::ExpCurr::ParamValues expCurrParams(5.0); // current 
    WeightUpdateModels::StaticPulse::VarValues staticSynapseInitVals(uninitialisedVar());


    //----------------------- NEURON POPULATION -----------------------------------------//
    // HD network
    model.addNeuronPopulation<NeuronModels::LIF>("Obs_pop",  8, HD_params, InitVals);
    model.addNeuronPopulation<NeuronModels::LIF>("Motor_pop",  2, HD_params, InitVals);
    model.addNeuronPopulation<NeuronModels::LIF>("Speed_pop",  1, HD_params, InitVals);

    // self excitatory speed connection
    //model.addSynapsePopulation<WeightUpdateModels::StaticPulse, PostsynapticModels::ExpCurr>(
    //    "Speed_ex",SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
    //    "Speed_pop", "Speed_pop", 
    //    {}, staticSynapseInitVals, expCurrParams,{});

    // obstacle syn inhibiting speed pop
    model.addSynapsePopulation<WeightUpdateModels::StaticPulse, PostsynapticModels::ExpCurr>(
        "Obs_inh_speed",SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "Obs_pop", "Speed_pop", 
        {}, staticSynapseInitVals, expCurrParams,{});

   
    model.addSynapsePopulation<WeightUpdateModels::StaticPulse, PostsynapticModels::ExpCurr>(
        "Obs_in",SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "Obs_pop", "Obs_pop", 
        {}, staticSynapseInitVals, expCurrParams,{});

    model.addSynapsePopulation<WeightUpdateModels::StaticPulse, PostsynapticModels::ExpCurr>(
        "Obs_motor",SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "Obs_pop", "Motor_pop", 
        {}, staticSynapseInitVals, expCurrParams,{});
    
    model.addSynapsePopulation<WeightUpdateModels::StaticPulse, PostsynapticModels::ExpCurr>(
        "Motor_inh",SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "Motor_pop", "Motor_pop", 
        {}, staticSynapseInitVals, expCurrParams,{});

    // current to obs pop
    model.addCurrentSource<DE>("Obs_curr", "Obs_pop", {}, stimVars);

    // current to obs pop
    model.addCurrentSource<DE>("Speed_curr", "Speed_pop", {}, stimVars);
    
}






