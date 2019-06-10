#pragma once

// BoB robotics includes
#include "common/pose.h"
#include "robots/uav.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <array>
#include <utility>

namespace BoBRobotics {
namespace Robots {
using namespace std::literals;

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
                  Vicon::UDPClient<Vicon::ObjectDataVelocity> &vicon,
                  const Bounds::Range &xRange = { -0.5_m, 0.5_m },
                  const Bounds::Range &yRange = { -0.5_m, 0.5_m },
                  const Bounds::Range &zRange = { 0_m, 1_m });

    void update();
    void setYaw(degree_t yaw);
    void setWaypoint(const Vector3<meter_t> &waypoint);

    Vicon::UDPClient<Vicon::ObjectDataVelocity> &m_Vicon;
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
