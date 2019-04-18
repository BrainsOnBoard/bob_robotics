#pragma once

// Third-party includes
#include "third_party/units.h"

//------------------------------------------------------------------------
// SimParams
//------------------------------------------------------------------------
namespace SimParams
{
    using namespace units::angle;
    using namespace units::length;
    using namespace units::literals;

    // Testing parameters
    constexpr degree_t scanAngle = 120.0_deg;
    constexpr degree_t scanStep = 2.0_deg;
    constexpr degree_t spinStep = 0.5_deg;
    constexpr meter_t snapshotDistance = 10.0_cm;
    constexpr meter_t errorDistance = 20.0_cm;
    constexpr unsigned int testStepLimit = 1000;

    constexpr unsigned int numScanSteps = (unsigned int)(scanAngle / scanStep);

    constexpr unsigned int numVectorFieldSteps = (unsigned int)(360.0_deg / scanStep);

    // Rendering parameters
    // What colour should the ground be?
    constexpr float groundColour[] = {0.898f, 0.718f, 0.353f};

    // What colour should the brightest tussocks be?
    constexpr float worldColour[] = {0.0f, 1.0f, 0.0f};

    // From these calculate display size
    constexpr unsigned int displayRenderWidth = 688;
    constexpr unsigned int displayRenderHeight = 160;

    // Ant parameters
    // How fast does the ant move each timestep?
    constexpr degree_t antTurnStep = 1.0_deg;
    constexpr meter_t antMoveStep = 0.01_m;
}
