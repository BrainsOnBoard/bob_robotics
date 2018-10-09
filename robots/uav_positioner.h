#pragma once

// BoB robotics includes
#include "uav.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;
using namespace units::angle;
using namespace units::math;

class UAVPositioner
{
public:
    UAVPositioner(UAV &uav, float yawSpeed, radian_t angleThreshold = 5_deg)
      : m_UAV(uav)
      , m_YawSpeed(yawSpeed)
      , m_AngleThreshold(angleThreshold)
    {}

    template<typename ObjectData>
    bool update(const ObjectData &object)
    {
        const auto att = object.template getAttitude<>();
        if (abs(att[0]) < m_AngleThreshold) {
            return false;
        } else {
            if (att[0] < 0_rad) {
                m_UAV.setYawSpeed(m_YawSpeed);
            } else {
                m_UAV.setYawSpeed(-m_YawSpeed);
            }
            return true;
        }
    }

private:
    UAV &m_UAV;
    float m_YawSpeed;
    radian_t m_AngleThreshold;

}; // UAVPositioner
} // Robots
} // BoBRobotics