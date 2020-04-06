#pragma once

// GeNN includes
#include "modelSpec.h"

namespace BoBRobotics {
namespace GeNNModels {

//----------------------------------------------------------------------------
// BoBRobotics::GeNNModels::Sigmoid
//----------------------------------------------------------------------------
//! Non-spiking sigmoid unit
class Sigmoid : public NeuronModels::Base
{
public:
    DECLARE_MODEL(Sigmoid, 2, 1);

    SET_SIM_CODE(
        "$(r) = 1.0 / (1.0 + exp(-(($(a) * $(Isyn)) - $(b))));\n"
    );

    SET_PARAM_NAMES({
        "a",        // Multiplicative scale
        "b"});      // Additive scale

    SET_VARS({{"r", "scalar"}});
};
IMPLEMENT_MODEL(Sigmoid);
} // GeNNModels
} // BoBRobotics

