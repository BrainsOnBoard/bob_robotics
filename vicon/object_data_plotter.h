#pragma once

// BoB robotics includes
#include "udp.h"

// Third-party includes
#include "../third_party/matplotlibcpp.h"
#include "../third_party/units.h"

// Standard C++ includes
#include <thread>

namespace BoBRobotics {
namespace Vicon {
template<typename BaseObjectDataType = ObjectData>
class ObjectDataPlotter
  : public ObjectData {
public:
    ObjectDataPlotter() : m_PlotThread(run, this)
    {}

    void updatePlot() const
    {
        const auto position = getPosition<>();
        const auto attitude = getAttitude<>();
        // std::cout << "x: " << position[0] << ", y: " << position[1] << std::endl;
        const std::vector<double> vx{ position[0].value() };
        const std::vector<double> vy{ position[1].value() };
        const std::vector<double> vu{ units::math::cos(attitude[0]) };
        const std::vector<double> vv{ units::math::sin(attitude[0]) };
        const std::vector<int> x0 { 0 }, y0 { 0 };
        matplotlibcpp::clf();
        matplotlibcpp::plot(x0, y0, "r+");
        matplotlibcpp::quiver(vx, vy, vu, vv);
        matplotlibcpp::xlim(-2500, 2500);
        matplotlibcpp::ylim(-2500, 2500);
        matplotlibcpp::pause(0.01);
    }

private:
    std::thread m_PlotThread;

    static void run(const ObjectDataPlotter *plotter)
    {
        while (true)
            plotter->updatePlot();
    }
}; // ObjectDataPlotter
} // Vicon
} // BoBRobotics