#pragma once

// BoB robotics includes
#include "../common/pose.h"
#include "../common/stopwatch.h"
#include "tank.h"

// Third-party includes
#include "../third_party/units.h"
#include <utility> 
namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

template<typename LengthUnit = units::length::millimeter_t,
         typename AngleUnit = units::angle::degree_t>
class GazeboTank
  : public Tank
{
    using millimeter_t = units::length::millimeter_t;
    using meters_per_second_t = units::velocity::meters_per_second_t;

public:
    GazeboTank(const meters_per_second_t maximumSpeed)
      : m_MaximumSpeed(maximumSpeed)
    {}


    const auto &getWheelSpeeds(){
        return m_WheelSpeeds;
    }
    virtual meters_per_second_t getAbsoluteMaximumSpeed() const override
    {
        return m_MaximumSpeed;
    }

    virtual void tank(float left, float right) override
    {
        BOB_ASSERT(left >= -1.f && left <= 1.f);
        BOB_ASSERT(right >= -1.f && right <= 1.f);
        m_WheelSpeeds.first = left * m_MaximumSpeed;
        m_WheelSpeeds.second = right * m_MaximumSpeed;
    }

private:

    const meters_per_second_t m_MaximumSpeed;
    // meters_per_second_t m_Right{}, m_Left{};
    std::pair <meters_per_second_t, meters_per_second_t> m_WheelSpeeds;

}; // GazeboTank
} // Robots
} // BoBRobotics
