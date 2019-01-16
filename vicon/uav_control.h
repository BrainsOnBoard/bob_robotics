#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../robots/uav.h"
#include "udp.h"

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <utility>

namespace BoBRobotics {
namespace Vicon {
using namespace std::literals;
using namespace units::literals;

class UAVControl
{
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;
    using degree_t = units::angle::degree_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;
    using Limits = std::pair<meter_t, meter_t>;

public:
    // limits of the room - default to a (hopefully) safe 1m^3
    struct Bounds
    {
        Limits x{ -0.5_m, 0.5_m };
        Limits y{ -0.5_m, 0.5_m };
        Limits z{ 0_m, 1_m };
    };

    UAVControl()
    {}

    UAVControl(const Bounds &roomBounds)
      : m_RoomBounds(roomBounds)
    {}

    void setPose(Robots::UAV &drone, const ObjectDataVelocity &objectData,
                 const Vector3<meter_t> &waypoint, const degree_t yaw)
    {
        using namespace units::math;

        if (waypoint[0] < m_RoomBounds.x.first || waypoint[0] > m_RoomBounds.x.second ||
            waypoint[1] < m_RoomBounds.y.first || waypoint[1] > m_RoomBounds.y.second ||
            waypoint[2] < m_RoomBounds.z.first || waypoint[2] > m_RoomBounds.z.second) {
            std::cerr << "WARNING: Attempted to move outside of room bounds" << std::endl;
            return;
        }

        // Get current pose and velocity from Vicon system
        const auto position = objectData.getPosition<meter_t>();
        const auto attitude = objectData.getAttitude<radian_t>();
        const auto velocity = objectData.getVelocity();

        // calc distance to waypoint
        Vector3<meter_t> posDiff;
        std::transform(waypoint.cbegin(), waypoint.cend(), position.cbegin(), posDiff.begin(),
            [](auto p0, auto p1) -> meter_t {
                return p0 - p1;
            });

        Vector3<meters_per_second_t> targetVelocity;
        Vector3<meters_per_second_t> acceleration;

        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << posDiff[0] << " " << float(posDiff[1]) << "," << std::endl;

        // calculate velocity setpoint
        for (size_t i = 0; i < 3; ++i) {
            if (posDiff[i] > 0.4_m) {
                targetVelocity[i] = 0.5_mps;
            } else if (posDiff[i] < -0.4_m) {
                targetVelocity[i] = -0.5_mps;
            } else {
                targetVelocity[i] = posDiff[i] * 1.25 / 1_s;
            }

            // smooth the v setpoint to avoid jerks
            m_VelocitySetPoint[i] = m_VelocitySetPoint[i] * 0.9 + targetVelocity[i] * 0.1;

            // calculate acceleration -- **TODO**: This is actually a difference of velocities
            acceleration[i] = m_OldVelocity[i] - velocity[i];
        }

        // Calculate integral terms
        for (size_t i = 0; i < 3; ++i) {
            m_IntegralTerm[i] += m_VelocitySetPoint[i] - velocity[i];
        }

        float newUp;
        if (m_VelocitySetPoint[2] - velocity[2] > 0_mps) {
            newUp = ((0.2 * m_IntegralTerm[2] + 35.0 * (m_VelocitySetPoint[2] - velocity[2]) + 0.25 * acceleration[2]) / 150.0).value();
        } else {
            newUp = ((0.2 * m_IntegralTerm[2] + 35.0 * (m_VelocitySetPoint[2] - velocity[2]) - 0.25 * acceleration[2]) / 150.0).value();
        }
        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_VelocitySetPoint[2] << " " << float(velocity[2]) << "," << std::endl;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << newUp << ",";
        drone.setVerticalSpeed(newUp);

        const double pRoll = (-100.0 * (-sin(attitude[0]) * (m_VelocitySetPoint[0] - velocity[0]) + cos(attitude[0]) * (m_VelocitySetPoint[1] - velocity[1]))).value();
        const double iRoll = (-0.1 * (-sin(attitude[0]) * m_IntegralTerm[0] + cos(attitude[0]) * m_IntegralTerm[1])).value();
        const double dRoll = (-10.0 * (-sin(attitude[0]) * acceleration[0] + cos(attitude[0]) * acceleration[1])).value();
        const float newRoll = (pRoll + dRoll + iRoll) / 100.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << newRoll << ",";
        drone.setRoll(newRoll);

        const double pPitch = (-100.0 * (cos(attitude[0]) * (m_VelocitySetPoint[0] - velocity[0]) + sin(attitude[0]) * (m_VelocitySetPoint[1] - velocity[1]))).value();
        const double iPitch = (-0.1 * (cos(attitude[0]) * m_IntegralTerm[0] + sin(attitude[0]) * m_IntegralTerm[1])).value();
        const double dPitch = (-10.0 * (cos(attitude[0]) * acceleration[0]) + sin(attitude[0]) * acceleration[1]).value();
        const float newPitch = (pPitch + dPitch + iPitch) / 100.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << newPitch << ",";
        drone.setPitch(newPitch);

        const float newYawSpeed = -atan2(sin(yaw - attitude[0]), cos(yaw - attitude[0])).value(); // was 20.75
        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << newYawSpeed << ",";
        drone.setYawSpeed(newYawSpeed);

        //std::cout << std::endl;

        m_OldVelocity = velocity;
    }

    Bounds m_RoomBounds;
    Vector3<meters_per_second_t> m_VelocitySetPoint, m_OldVelocity;
    Vector3<meters_per_second_t> m_IntegralTerm{ 0_mps, 0_mps, 300_mps };
}; // UAVControl
} // Vicon
} // BoBRobotics
