#pragma once

// BoB robotics includes
#include "../common/plot_agent.h"
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

template<typename BaseObjectType = Object>
class ObjectPlotter
  : public Object
{
public:
    ObjectPlotter() = default;
    ObjectPlotter(const ObjectPlotter &&o2)
    {
        const auto pos = o2.getPosition<>();
        const auto att = o2.getAttitude<>();
        BaseObjectType::update(o2.getFrameNumber(), pos[0], pos[1], pos[2], att[0], att[1], att[2]);
    }

    void update(uint32_t frameNumber, millimeter_t x, millimeter_t y, millimeter_t z, radian_t yaw, radian_t pitch, radian_t roll)
    {
        BaseObjectType::update(frameNumber, x, y, z, yaw, pitch, roll);
        m_NewData = true;
    }

    bool update()
    {
        // Atomically retrieve value of m_NewData and set to false
        if (!m_NewData.exchange(false)) {
            return false;
        }

        // Update plot
        plotAgent(*this, { -2500, 2500 }, { -2500, 2500 });
        matplotlibcpp::pause(0.01);
        return true;
    }
private:
    std::atomic<bool> m_NewData{ false };
}; // ObjectPlotter
} // Vicon
} // BoBRobotics