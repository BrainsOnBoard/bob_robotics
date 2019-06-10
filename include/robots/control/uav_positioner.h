#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "robots/uav.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <array>
#include <utility>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

// limits of the room - default to a (hopefully) safe 1m^3
struct Bounds
{
    using Range = std::pair<units::length::meter_t, units::length::meter_t>;
    Range x, y, z;
};

class UAVPositioner
{
    using meter_t = units::length::meter_t;
    using degree_t = units::angle::degree_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    UAVPositioner(Robots::UAV &drone,
                  const Bounds::Range &xRange = { -0.5_m, 0.5_m },
                  const Bounds::Range &yRange = { -0.5_m, 0.5_m },
                  const Bounds::Range &zRange = { 0_m, 1_m });

    void update(const Pose3<meter_t, degree_t> &pose,
                const std::array<meters_per_second_t, 3> &velocity);
    void setYaw(degree_t yaw);
    void setWaypoint(const Vector3<meter_t> &waypoint);

    Robots::UAV &m_Drone;
    Bounds m_RoomBounds;
    Vector3<meter_t> m_Waypoint;
    degree_t m_Yaw = 0.0_deg;
    std::array<float, 3> m_VSetPoint = { { 0, 0, 0 } };
    std::array<meters_per_second_t, 3> m_OldVelocity = { { 0_mps, 0_mps, 0_mps } };
    std::array<float, 3> m_IntegralTerm = { { 0, 0, 300.0f } };
}; // UAVPositioner
} // Robots
} // BoBRobotics
