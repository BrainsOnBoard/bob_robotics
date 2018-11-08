#pragma once

// BoB robotics includes
#include "pose.h"

// Third-party includes
#include "../third_party/matplotlibcpp.h"
#include "../third_party/units.h"

// Standard C++ includes
#include <string>

namespace BoBRobotics {
    template<typename LengthUnit, typename AgentType>
    void plotAgent(AgentType &agent,
                   const LengthUnit xLower,
                   const LengthUnit xUpper,
                   const LengthUnit yLower,
                   const LengthUnit yUpper)
    {
        // Get agent's pose
        const auto position = agent.template getPosition<LengthUnit>();
        const auto attitude = agent.template getAttitude<>();

        // Get vector components
        const std::vector<double> vx{ position[0].value() };
        const std::vector<double> vy{ position[1].value() };
        const std::vector<double> vu{ units::math::cos(attitude[0]) };
        const std::vector<double> vv{ units::math::sin(attitude[0]) };

        // Origin
        const std::vector<int> x0 { 0 }, y0 { 0 };

        // Update plot
        namespace plt = matplotlibcpp;
        plt::plot(x0, y0, "r+");
        plt::quiver(vx, vy, vu, vv);
        plt::xlim(xLower.value(), xUpper.value());
        plt::ylim(yLower.value(), yUpper.value());

        // Label axes with the length unit
        const std::string abbrev = units::abbreviation(xLower);
        plt::xlabel("x (" + abbrev + ")");
        plt::ylabel("y (" + abbrev + ")");
    }
} // BoBRobotics
