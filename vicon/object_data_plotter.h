#pragma once

// BoB robotics includes
#include "udp.h"

// Third-party includes
#include "../third_party/matplotlibcpp.h"
#include "../third_party/units.h"

// Standard C++ includes
#include <atomic>
#include <chrono>
#include <thread>

namespace BoBRobotics {
namespace Vicon {
using namespace std::literals;

template<typename BaseObjectDataType = ObjectData>
class ObjectDataPlotter
  : public ObjectData
{
public:
    ObjectDataPlotter() = default;
    ObjectDataPlotter(const ObjectDataPlotter &) {}

    void update(uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z, radian_t yaw, radian_t pitch, radian_t roll)
    {
        BaseObjectDataType::update(frameNumber, x, y, z, yaw, pitch, roll);
        m_NewData = true;
    }

    bool update()
    {
        // Atomically retrieve value of m_NewData and set to false
        if (!m_NewData.exchange(false)) {
            return false;
        }

        // Update plot
        const auto position = getPosition<>();
        const auto attitude = getAttitude<>();
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
        return true;
    }
private:
    std::atomic<bool> m_NewData{ false };
}; // ObjectDataPlotter
} // Vicon
} // BoBRobotics