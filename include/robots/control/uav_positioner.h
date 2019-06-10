#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"
#include "robots/uav.h"
#include "vicon/udp.h"

// Standard C++ includes
#include <array>
#include <iomanip>
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
                  const Bounds::Range &zRange = { 0_m, 1_m })
      : m_Vicon(vicon)
      , m_Drone(drone)
    {
        // Check input values are sane
        BOB_ASSERT(xRange.first < xRange.second);
        BOB_ASSERT(yRange.first < yRange.second);
        BOB_ASSERT(zRange.first < zRange.second);
    }

    void printStatus()
    {
        if (m_Vicon.getNumObjects() == 1) {
            auto objectData = m_Vicon.getObjectData(0);
            const auto position = objectData.getPosition<>();
            const auto attitude = objectData.getAttitude<degree_t>();

            for (int i = 0; i < 3; ++i) {
                std::cout << std::setw(5) << std::fixed << std::setprecision(2) << position[i] / 1000.0 << ", ";
            }
            for (int i = 0; i < 3; ++i) {
                std::cout << std::setw(6) << std::fixed << std::setprecision(2) << attitude[i];
                if (i < 2)
                    std::cout << ", ";
            }

        } else {
            std::cout << "NO VICON DATA";
        }
        std::cout << "]" << std::endl;
    }

    void update()
    {
        // update control
        auto objectData = m_Vicon.getObjectData(0);
        const auto position = objectData.getPosition<>();
        const auto attitude = objectData.getAttitude<degree_t>();
        const auto &velocity = objectData.getVelocity();

        // calc distance to m_Waypoint
        std::array<float, 3> p_diff;
        for (int i = 0; i < 3; ++i) {
            p_diff[i] = m_Waypoint[i].value() - position[i].value() / 1000.0;
        }

        std::array<float, 3> targetVSetpoint;
        std::array<meters_per_second_t, 3> acceleration;

        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << p_diff[0] << " " << float(p_diff[1]) << "," << std::endl;

        // calculate velocity setpoint
        float speed = 1.0;
        for (int i = 0; i < 3; ++i) {
            if (p_diff[i] > 0.4) {
                targetVSetpoint[i] = 0.5 * speed;
            } else if (p_diff[i] < -0.4) {
                targetVSetpoint[i] = -0.5 * speed;
            } else {
                targetVSetpoint[i] = p_diff[i] * 1.25 * speed;
            }
            // smooth the v setpoint to avoid jerks
            m_VSetPoint[i] = m_VSetPoint[i] * 0.9 + targetVSetpoint[i] * 0.1;
            // calculate acceleration
            acceleration[i] = m_OldVelocity[i] - velocity[i];
        }

        // CONTROL
        for (int i = 0; i < 3; ++i) {
            // calculate Integral m_IntegralTerm
            m_IntegralTerm[i] = m_IntegralTerm[i] + (m_VSetPoint[i] - float(velocity[i]));
        }

        float z_control;
        if (m_VSetPoint[2] - float(velocity[2]) > 0) {
            z_control = (0.2 * m_IntegralTerm[2] + 35.0 * (m_VSetPoint[2] - float(velocity[2])) + 0.25 * float(acceleration[2])) / 150.0;
        } else {
            z_control = (0.2 * m_IntegralTerm[2] + 35.0 * (m_VSetPoint[2] - float(velocity[2])) - 0.25 * float(acceleration[2])) / 150.0;
        }
        //std::cout << std::setw(6) << std::fixed<< std::setprecision(4) << m_VSetPoint[2] << " " << float(velocity[2]) << "," << std::endl;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << z_control << ",";
        m_Drone.setVerticalSpeed(z_control);

        float roll_control;
        float p_roll = -100.0 * (-sin(float(attitude[0] / 180.0 * M_PI)) * (m_VSetPoint[0] - float(velocity[0])) + cos(float(attitude[0] / 180.0 * M_PI)) * (m_VSetPoint[1] - float(velocity[1])));
        float d_roll = -10.0 * (-sin(float(attitude[0] / 180.0 * M_PI)) * float(acceleration[0]) + cos(float(attitude[0] / 180.0 * M_PI)) * float(acceleration[1]));
        float i_roll = -0.1 * (-sin(float(attitude[0] / 180.0 * M_PI)) * m_IntegralTerm[0] + cos(float(attitude[0] / 180.0 * M_PI)) * m_IntegralTerm[1]);
        roll_control = (p_roll + d_roll + i_roll) / 100.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << roll_control << ",";
        m_Drone.setRoll(roll_control);

        float pitch_control;
        float p_pitch = -100.0 * (cos(float(attitude[0] / 180.0 * M_PI)) * (m_VSetPoint[0] - float(velocity[0])) + sin(float(attitude[0] / 180.0 * M_PI)) * (m_VSetPoint[1] - float(velocity[1])));
        float d_pitch = -10.0 * (cos(float(attitude[0] / 180.0 * M_PI)) * float(acceleration[0]) + sin(float(attitude[0] / 180.0 * M_PI)) * float(acceleration[1]));
        float i_pitch = -0.1 * (cos(float(attitude[0] / 180.0 * M_PI)) * m_IntegralTerm[0] + sin(float(attitude[0] / 180.0 * M_PI)) * m_IntegralTerm[1]);
        pitch_control = (p_pitch + d_pitch + i_pitch) / 100.0;

        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << pitch_control << ",";
        m_Drone.setPitch(pitch_control);

        float yaw_control = (-1.0f) * (180.0 / M_PI) * float(units::math::atan2(units::math::sin(m_Yaw - attitude[0]), units::math::cos(m_Yaw - attitude[0]))); // was 20.75
        //std::cout << std::setw(6) << std::fixed << std::setprecision(4) << yaw_control << ",";
        m_Drone.setYawSpeed(yaw_control);

        //std::cout << std::endl;

        m_OldVelocity = velocity;
    }

    void setYaw(degree_t yaw)
    {
        m_Yaw = yaw;
    }

    void setWaypoint(const Vector3<meter_t> &waypoint)
    {
        // Check that the user is not trying to move out of our room bounds
        BOB_ASSERT(waypoint.x() >= m_RoomBounds.x.first && waypoint.x() <= m_RoomBounds.x.second);
        BOB_ASSERT(waypoint.y() >= m_RoomBounds.y.first && waypoint.y() <= m_RoomBounds.y.second);
        BOB_ASSERT(waypoint.z() >= m_RoomBounds.z.first && waypoint.z() <= m_RoomBounds.z.second);

        // Store waypoint
        m_Waypoint = waypoint;
    }

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
