#pragma once

// Third-party includes
#include "../third_party/matplotlibcpp.h"
#include "../third_party/units.h"

// Standard C++ includes
#include <array>

namespace BoBRobotics {
    template<typename AgentType>
    void plotAgent(const AgentType &agent,
                   const std::array<double, 2> &xlim,
                   const std::array<double, 2> &ylim)
    {
        // Update plot
        const auto position = agent.template getPosition<>();
        const auto attitude = agent.template getAttitude<>();
        const std::vector<double> vx{ position[0].value() };
        const std::vector<double> vy{ position[1].value() };
        const std::vector<double> vu{ units::math::cos(attitude[0]) };
        const std::vector<double> vv{ units::math::sin(attitude[0]) };
        const std::vector<int> x0 { 0 }, y0 { 0 };
        matplotlibcpp::plot(x0, y0, "r+");
        matplotlibcpp::quiver(vx, vy, vu, vv);
        matplotlibcpp::xlim(xlim[0], xlim[1]);
        matplotlibcpp::ylim(ylim[0], ylim[1]);
    }
} // BoBRobotics
