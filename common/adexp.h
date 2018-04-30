#pragma once

// GeNN includes
#include "modelSpec.h"

//----------------------------------------------------------------------------
// AdExp
//----------------------------------------------------------------------------
//! Adaptive exponential - solved using RK4
class AdExp : public NeuronModels::Base
{
public:
    DECLARE_MODEL(AdExp, 11, 2);

    SET_SIM_CODE(
        "#define DV(V, W) (1.0 / $(c)) * ((-$(gL) * ((V) - $(eL))) + ($(gL) * $(deltaT) * exp(((V) - $(vThresh)) / $(deltaT))) + i - (W))\n"
        "#define DW(V, W) (1.0 / $(tauW)) * (($(a) * (V - $(eL))) - W)\n"
        "const scalar i = $(Isyn) + $(iOffset);\n"
        "// If voltage is above artificial spike height\n"
        "if($(v) >= $(vSpike)) {\n"
        "   $(v) = $(vReset);\n"
        "}\n"
        "// Calculate RK4 terms\n"
        "const scalar v1 = DV($(v), $(w));\n"
        "const scalar w1 = DW($(v), $(w));\n"
        "const scalar v2 = DV($(v) + (DT * 0.5 * v1), $(w) + (DT * 0.5 * w1));\n"
        "const scalar w2 = DW($(v) + (DT * 0.5 * v1), $(w) + (DT * 0.5 * w1));\n"
        "const scalar v3 = DV($(v) + (DT * 0.5 * v2), $(w) + (DT * 0.5 * w2));\n"
        "const scalar w3 = DW($(v) + (DT * 0.5 * v2), $(w) + (DT * 0.5 * w2));\n"
        "const scalar v4 = DV($(v) + (DT * v3), $(w) + (DT * w3));\n"
        "const scalar w4 = DW($(v) + (DT * v3), $(w) + (DT * w3));\n"
        "// Update V\n"
        "$(v) += (DT / 6.0) * (v1 + (2.0f * (v2 + v3)) + v4);\n"
        "// If we're not above peak, update w\n"
        "// **NOTE** it's not safe to do this at peak as wn may well be huge\n"
        "if($(v) <= -40.0) {\n"
        "   $(w) += (DT / 6.0) * (w1 + (2.0 * (w2 + w3)) + w4);\n"
        "}\n"
    );

    SET_THRESHOLD_CONDITION_CODE("$(v) > -40");

    SET_RESET_CODE(
        "// **NOTE** we reset v to arbitrary plotting peak rather than to actual reset voltage\n"
        "$(v) = $(vSpike);\n"
        "$(w) += ($(b) * 1000.0);");

    SET_PARAM_NAMES({
        "c",        // Membrane capacitance [pF]
        "gL",       // Leak conductance [nS]
        "eL",       // Leak reversal potential [mV]
        "deltaT",   // Slope factor [mV]
        "vThresh",  // Threshold voltage [mV]
        "vSpike",   // Artificial spike height [mV]
        "vReset",   // Reset voltage [mV]
        "tauW",     // Adaption time constant
        "a",        // Subthreshold adaption [nS]
        "b",        // Spike-triggered adaptation [nA]
        "iOffset",  // Offset current
    });

    SET_VARS({{"v", "scalar"}, {"w", "scalar"}});
};
IMPLEMENT_MODEL(AdExp);