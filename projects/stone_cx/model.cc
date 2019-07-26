#include "modelSpec.h"

// BoB robotics includes
#include "../../include/genn_models/sigmoid.h"

// Stone CX includes
#include "parameters.h"

using namespace BoBRobotics;
using namespace BoBRobotics::StoneCX;

//---------------------------------------------------------------------------
// Continuous
//---------------------------------------------------------------------------
class Continuous : public WeightUpdateModels::Base
{
public:
    DECLARE_MODEL(Continuous, 0, 1);

    SET_VARS({{"g", "scalar"}});

    SET_SYNAPSE_DYNAMICS_CODE("$(addToInSyn, $(g) * $(r_pre));\n");
};
IMPLEMENT_MODEL(Continuous);

//---------------------------------------------------------------------------
// TN2Linear
//---------------------------------------------------------------------------
class TN2Linear : public NeuronModels::Base
{
public:
    DECLARE_MODEL(TN2Linear,0, 2);

    // **NOTE** this comes from https://github.com/InsectRobotics/path-integration/blob/master/cx_rate.py#L170-L173 rather than the methods section
    SET_SIM_CODE("$(r) = fmin(1.0, fmax($(speed), 0.0));\n");

    SET_VARS({{"r", "scalar"},
              {"speed", "scalar"}});
};
IMPLEMENT_MODEL(TN2Linear);

//---------------------------------------------------------------------------
// TLSigmoid
//---------------------------------------------------------------------------
class TLSigmoid : public NeuronModels::Base
{
public:
    DECLARE_MODEL(TLSigmoid, 2, 2);

    SET_SIM_CODE(
        "const scalar iTL = cos($(preferredAngle) - $(headingAngle));\n"
        "$(r) = 1.0 / (1.0 + exp(-(($(a) * iTL) - $(b))));\n"
    );

    SET_PARAM_NAMES({
        "a",        // Multiplicative scale
        "b"});      // Additive scale

    SET_VARS({{"r", "scalar"},
              {"preferredAngle", "scalar"}});

    SET_EXTRA_GLOBAL_PARAMS({{"headingAngle", "scalar"}});
};
IMPLEMENT_MODEL(TLSigmoid);

//----------------------------------------------------------------------------
// CPU4Sigmoid
//----------------------------------------------------------------------------
//! Non-spiking sigmoid unit
class CPU4Sigmoid : public NeuronModels::Base
{
public:
    DECLARE_MODEL(CPU4Sigmoid, 4, 2);

    SET_SIM_CODE(
        "$(i) += $(h) * fmin(1.0, fmax($(Isyn), 0.0));\n"
        "$(i) -= $(h) * $(k);\n"
        "$(i) = fmin(1.0, fmax($(i), 0.0));\n"
        "$(r) = 1.0 / (1.0 + exp(-(($(a) * $(i)) - $(b))));\n"
    );

    SET_PARAM_NAMES({
        "a",        // Multiplicative scale
        "b",        // Additive scale
        "h",        // Input scale
        "k"});      // Offset current

    SET_VARS({{"r", "scalar"},
              {"i", "scalar"}});
};
IMPLEMENT_MODEL(CPU4Sigmoid);

//----------------------------------------------------------------------------
// PreferredAngle
//----------------------------------------------------------------------------
class PreferredAngle : public InitVarSnippet::Base
{
public:
    DECLARE_SNIPPET(PreferredAngle, 0);

    SET_CODE(
        "$(value) = 0.785398163 * (scalar)($(id) % 8);");
};
IMPLEMENT_SNIPPET(PreferredAngle);

//----------------------------------------------------------------------------
// TBToTB
//----------------------------------------------------------------------------
class TBToTB : public InitVarSnippet::Base
{
public:
    DECLARE_SNIPPET(TBToTB, 1);

    SET_CODE(
        "const scalar preferredI = 0.785398163 * (scalar)($(id_pre) % 8);\n"
        "const scalar preferredJ = 0.785398163 * (scalar)($(id_post) % 8);\n"
        "const scalar w = (cos(preferredI - preferredJ) - 1.0) / 2.0;\n"
        "$(value) = w * $(c);");

    SET_PARAM_NAMES({"c"});
};
IMPLEMENT_SNIPPET(TBToTB);

//----------------------------------------------------------------------------
// TBToCPU
//----------------------------------------------------------------------------
class TBToCPU : public InitSparseConnectivitySnippet::Base
{
public:
    DECLARE_SNIPPET(TBToCPU, 0);

    SET_ROW_BUILD_CODE(
        "$(addSynapse, $(id_pre));\n"
        "$(addSynapse, $(id_pre) + 8);\n"
        "$(endRow);\n");

    SET_MAX_ROW_LENGTH(2);
    SET_MAX_COL_LENGTH(1);
};
IMPLEMENT_SNIPPET(TBToCPU);

//----------------------------------------------------------------------------
// CL1ToTB1
//----------------------------------------------------------------------------
class CL1ToTB1 : public InitSparseConnectivitySnippet::Base
{
public:
    DECLARE_SNIPPET(CL1ToTB1, 0);

    SET_ROW_BUILD_CODE(
        "$(addSynapse, $(id_pre) % 8);\n"
        "$(endRow);\n");

    SET_MAX_ROW_LENGTH(1);
    SET_MAX_COL_LENGTH(1);
};
IMPLEMENT_SNIPPET(CL1ToTB1);

//----------------------------------------------------------------------------
// PontineToCPU1
//----------------------------------------------------------------------------
class PontineToCPU1 : public InitSparseConnectivitySnippet::Base
{
public:
    DECLARE_SNIPPET(PontineToCPU1, 0);

    SET_ROW_BUILD_CODE(
        "if($(id_pre) < 5) {\n"
        "   $(addSynapse, $(id_pre) + 11);\n"
        "}\n"
        "else if($(id_pre) < 8) {\n"
        "   $(addSynapse, $(id_pre) + 3);\n"
        "}\n"
        "else if($(id_pre) < 11) {\n"
        "   $(addSynapse, $(id_pre) - 3);\n"
        "}\n"
        "else {\n"
        "   $(addSynapse, $(id_pre) - 11);\n"
        "}\n"
        "$(endRow);\n");

    SET_MAX_ROW_LENGTH(1);
    SET_MAX_COL_LENGTH(1);
};
IMPLEMENT_SNIPPET(PontineToCPU1);

//----------------------------------------------------------------------------
// CPU4ToCPU1
//----------------------------------------------------------------------------
class CPU4ToCPU1 : public InitSparseConnectivitySnippet::Base
{
public:
    DECLARE_SNIPPET(CPU4ToCPU1, 0);

    SET_ROW_BUILD_CODE(
        "if($(id_pre) == 0) {\n"
        "   $(addSynapse, 15);\n"
        "}\n"
        "else if($(id_pre) < 8) {\n"
        "   $(addSynapse, $(id_pre) + 7);\n"
        "}\n"
        "else if($(id_pre) < 15) {\n"
        "   $(addSynapse, $(id_pre) - 7);\n"
        "}\n"
        "else {\n"
        "   $(addSynapse, 0);\n"
        "}\n"
        "$(endRow);\n");

    SET_MAX_ROW_LENGTH(1);
    SET_MAX_COL_LENGTH(1);
};
IMPLEMENT_SNIPPET(CPU4ToCPU1);

//----------------------------------------------------------------------------
// TN2CPU4
//----------------------------------------------------------------------------
class TN2CPU4 : public InitSparseConnectivitySnippet::Base
{
public:
    DECLARE_SNIPPET(TN2CPU4, 0);

    SET_ROW_BUILD_STATE_VARS({{"c", "unsigned int", 0}});
    SET_ROW_BUILD_CODE(
        "$(addSynapse, ($(id_pre) * 8) + c);\n"
        "c++;\n"
        "if(c >= 8) {\n"
        "   $(endRow);\n"
        "}\n");

    SET_MAX_ROW_LENGTH(8);
    SET_MAX_COL_LENGTH(1);
};
IMPLEMENT_SNIPPET(TN2CPU4);

void modelDefinition(NNmodel &model)
{
    model.setDT(1.0);
    model.setName("stone_cx");

    //---------------------------------------------------------------------------
    // Neuron parameters
    //---------------------------------------------------------------------------
    GeNNModels::Sigmoid::VarValues sigmoidInit(0.0);

    // TN2
    TN2Linear::VarValues tn2Init(
        0.0,    // r
        0.0);   // speed

    // TL
    TLSigmoid::ParamValues tlParams(
        6.8,    // Multiplicative scale
        3.0);   // Additive scale

    TLSigmoid::VarValues tlInit(
        0.0,                        // r
        initVar<PreferredAngle>()); // Preference angle (radians)

    // CL1
    GeNNModels::Sigmoid::ParamValues cl1Params(
        3.0,     // Multiplicative scale
        -0.5);   // Additive scale

    // TB1
    GeNNModels::Sigmoid::ParamValues tb1Params(
        5.0,    // Multiplicative scale
        0.0);   // Additive scale

    // CPU4
    CPU4Sigmoid::ParamValues cpu4Params(
        5.0,    // Multiplicative scale
        2.5,    // Additive scale
        0.0025, // Input scale
        0.125);   // Offset current **NOTE** this is the value from github

    CPU4Sigmoid::VarValues cpu4Init(
        0.0,    // r
        0.5);   // i

    // Pontine
    GeNNModels::Sigmoid::ParamValues pontineParams(
        5.0,     // Multiplicative scale
        2.5);   // Additive scale

    // CPU1 **NOTE** these are the values from https://github.com/InsectRobotics/path-integration/blob/master/cx_rate.py#L231-L232
    GeNNModels::Sigmoid::ParamValues cpu1Params(
        7.5,     // Multiplicative scale
        -1.0);   // Additive scale

    //---------------------------------------------------------------------------
    // Synapse parameters
    //---------------------------------------------------------------------------
    Continuous::VarValues continuousExcInit(1.0);

    Continuous::VarValues continuousInhInit(-1.0);

    Continuous::VarValues cl1TB1Init(1.0 - Parameters::c);
    Continuous::VarValues cpu4CPU1Init(0.5);
    Continuous::VarValues pontineCPU1Init(-0.5);
    Continuous::VarValues tb1TB1Init(initVar<TBToTB>(Parameters::c));

    //---------------------------------------------------------------------------
    // Neuron populations
    //---------------------------------------------------------------------------
    model.addNeuronPopulation<TN2Linear>("TN2", Parameters::numTN2, {}, tn2Init);
    model.addNeuronPopulation<TLSigmoid>("TL", Parameters::numTL, tlParams, tlInit);
    model.addNeuronPopulation<GeNNModels::Sigmoid>("CL1", Parameters::numCL1, cl1Params, sigmoidInit);
    model.addNeuronPopulation<GeNNModels::Sigmoid>("TB1", Parameters::numTB1, tb1Params, sigmoidInit);
    model.addNeuronPopulation<CPU4Sigmoid>("CPU4", Parameters::numCPU4, cpu4Params, cpu4Init);
    model.addNeuronPopulation<GeNNModels::Sigmoid>("Pontine", Parameters::numPontine, pontineParams, sigmoidInit);
    model.addNeuronPopulation<GeNNModels::Sigmoid>("CPU1", Parameters::numCPU1, cpu1Params, sigmoidInit);

    //---------------------------------------------------------------------------
    // Synapse populations
    //---------------------------------------------------------------------------
    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "TL_CL1", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "TL", "CL1",
        {}, continuousInhInit,
        {}, {},
        initConnectivity<InitSparseConnectivitySnippet::OneToOne>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "CL1_TB1", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "CL1", "TB1",
        {}, cl1TB1Init,
        {}, {},
        initConnectivity<CL1ToTB1>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "TB1_TB1", SynapseMatrixType::DENSE_INDIVIDUALG, NO_DELAY,
        "TB1", "TB1",
        {}, tb1TB1Init,
        {}, {});

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "CPU4_Pontine", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "CPU4", "Pontine",
        {}, continuousExcInit,
        {}, {},
        initConnectivity<InitSparseConnectivitySnippet::OneToOne>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "TB1_CPU4", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "TB1", "CPU4",
        {}, continuousInhInit,
        {}, {},
        initConnectivity<TBToCPU>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "TB1_CPU1", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "TB1", "CPU1",
        {}, continuousInhInit,
        {}, {},
        initConnectivity<TBToCPU>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "CPU4_CPU1", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "CPU4", "CPU1",
        {}, cpu4CPU1Init,
        {}, {},
        initConnectivity<CPU4ToCPU1>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "TN2_CPU4", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "TN2", "CPU4",
        {}, continuousExcInit,
        {}, {},
        initConnectivity<TN2CPU4>());

    model.addSynapsePopulation<Continuous, PostsynapticModels::DeltaCurr>(
        "Pontine_CPU1", SynapseMatrixType::SPARSE_GLOBALG, NO_DELAY,
        "Pontine", "CPU1",
        {}, pontineCPU1Init,
        {}, {},
        initConnectivity<PontineToCPU1>());
}
