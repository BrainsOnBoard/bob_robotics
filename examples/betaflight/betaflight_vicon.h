#pragma once

// BoB robotics includes
/*
This file provides a vicon based control overlay for the BetaFlight
drone.
*/

#include <iomanip>

#include "../../vicon/capture_control.h"
#include "../../vicon/udp.h"

namespace BoBRobotics {
namespace Robots {
// limits of the room - default to a (hopefully) safe 1m^3
struct RBounds
{
    float x[2] = { -0.5, 0.5 };
    float y[2] = { -0.5, 0.5 };
    float z[2] = { 0, 1 };
};

using namespace BoBRobotics::Vicon;

class BetaflightVicon
{
    using degree_t = units::angle::degree_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    BetaflightVicon(std::string device, int baud)
      : m_Drone(device, baud)
      , m_Vicon(51001)
    {

        std::this_thread::sleep_for(0.1ms);

        // start streaming status data
        m_Drone.subscribe();

        std::this_thread::sleep_for(0.1s);
    }

    void arm()
    {
        m_Drone.arm();
    }

    void disarm()
    {
        m_Drone.disarm();
    }

    void printStatus()
    {
        if (m_Drone.getArmState().size() > 0) {
            std::cout << m_Drone.getArmState() << std::endl;
        }
        std::cout << "[V:" << std::setw(4) << m_Drone.getVoltage() << " ";

        if (m_Vicon.getNumObjects() > 0) {
            const auto objectData = m_Vicon.getObjectData(0);
            const auto position = objectData.getPosition<>();
            const auto attitude = objectData.getAttitude<degree_t>();

            for (int i = 0; i < 3; ++i) {
                std::cout << std::setw(5) << std::fixed << std::setprecision(2) << float(position[i]) / 1000.0 << ", ";
            }
            for (int i = 0; i < 3; ++i) {
                std::cout << std::setw(6) << std::fixed << std::setprecision(2) << float(attitude[i]);
                if (i < 2)
                    std::cout << ", ";
            }

        } else {
            std::cout << "NO VICON DATA";
        }
        std::cout << "]" << std::endl;
    }

    void sendCommands(bool controlOn)
    {
        if (controlOn) {
            // update control
            const auto objectData = m_Vicon.getObjectData(0);
            const auto position = objectData.getPosition<>();
            const auto attitude = objectData.getAttitude<degree_t>();
            const auto velocity = objectData.getVelocity();

            // calc distance to m_Waypoint
            Vector3<float> p_diff;
            for (int i = 0; i < 3; ++i) {
                p_diff[i] = m_Waypoint[i] - float(position[i]) / 1000.0;
            }

            Vector3<float> targetVSetpoint;
            Vector3<meters_per_second_t> acceleration;

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

        // NOTE: this is not currently a safe way of detecting dropout of VICON!
        if (m_Vicon.getNumObjects() == 1) {
            m_Drone.sendCommands();
        }
        // wait so we do not overload the drone
        std::this_thread::sleep_for(10ms);
    }

    void setRoomBounds(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    {
        m_RoomBounds.x[0] = x_min;
        m_RoomBounds.x[1] = x_max;
        m_RoomBounds.y[0] = y_min;
        m_RoomBounds.y[1] = y_max;
        m_RoomBounds.z[0] = z_min;
        m_RoomBounds.z[1] = z_max;
    }

    void setYaw(degree_t yaw)
    {
        m_Yaw = yaw;
    }

    void setWaypoint(float x, float y, float z)
    {
        if (x < m_RoomBounds.x[0] || x > m_RoomBounds.x[1] || y < m_RoomBounds.y[0] || y > m_RoomBounds.y[1] || z < m_RoomBounds.z[0] || z > m_RoomBounds.z[1]) {
            std::cout << "Attempted to move outside of room bounds" << std::endl;
            return;
        } else {
            m_Waypoint[0] = std::max(m_RoomBounds.x[0], std::min(m_RoomBounds.x[1], x));
            m_Waypoint[1] = std::max(m_RoomBounds.y[0], std::min(m_RoomBounds.y[1], y));
            m_Waypoint[2] = std::max(m_RoomBounds.z[0], std::min(m_RoomBounds.z[1], z));
        }
    }

    BoBRobotics::Robots::BetaflightUAV m_Drone;
    UDPClient<ObjectDataVelocity> m_Vicon;
    RBounds m_RoomBounds;
    Vector3<float> m_Waypoint = { { 0, 0, 0 } };
    degree_t m_Yaw = 0.0_deg;
    Vector3<float> m_VSetPoint = { { 0, 0, 0 } };
    Vector3<meters_per_second_t> m_OldVelocity = { { 0_mps, 0_mps, 0_mps } };
    Vector3<float> m_IntegralTerm = { { 0, 0, 300.0f } };
}; // BetaflightUAV
} // namespace Robots
} // namespace BoBRobotics
