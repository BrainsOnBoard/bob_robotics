// BoB robotics includes
#include "common/macros.h"
#include "robots/control/uav_positioner.h"

using namespace std::literals;

namespace BoBRobotics {
namespace Robots {

UAVPositioner::UAVPositioner(Robots::UAV &drone,
                             const Bounds::Range &xRange,
                             const Bounds::Range &yRange,
                             const Bounds::Range &zRange)
  : m_Drone(drone)
{
    // Check input values are sane
    BOB_ASSERT(xRange.first < xRange.second);
    BOB_ASSERT(yRange.first < yRange.second);
    BOB_ASSERT(zRange.first < zRange.second);
}

void
UAVPositioner::update(const Pose3<meter_t, degree_t> &pose,
                      const std::array<meters_per_second_t, 3> &velocity)
{
    // update control
    const auto &position = pose.position();
    const auto &attitude = pose.attitude();

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

void
UAVPositioner::setYaw(degree_t yaw)
{
    m_Yaw = yaw;
}

void
UAVPositioner::setWaypoint(const Vector3<meter_t> &waypoint)
{
    // Check that the user is not trying to move out of our room bounds
    BOB_ASSERT(waypoint.x() >= m_RoomBounds.x.first && waypoint.x() <= m_RoomBounds.x.second);
    BOB_ASSERT(waypoint.y() >= m_RoomBounds.y.first && waypoint.y() <= m_RoomBounds.y.second);
    BOB_ASSERT(waypoint.z() >= m_RoomBounds.z.first && waypoint.z() <= m_RoomBounds.z.second);

    // Store waypoint
    m_Waypoint = waypoint;
}

} // Robots
} // BoBRobotics
