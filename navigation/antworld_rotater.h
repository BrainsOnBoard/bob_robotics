#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../libantworld/agent.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::AntWorldRotater
//------------------------------------------------------------------------
/*!
 * \brief A "rotater" for PerfectMemoryBase classes which uses OpenGL to rotate
 *        in AntWorld
 */
class AntWorldRotater
{
    using degree_t = units::angle::degree_t;

public:
    AntWorldRotater(const cv::Size &unwrapRes,
                    const cv::Mat &maskImage,
                    AntWorld::AntAgent &agent,
                    const degree_t yawStep = 1_deg,
                    const degree_t pitch = 0_deg,
                    const degree_t roll = 0_deg)
      : m_Agent(agent)
      , m_YawStep(yawStep)
      , m_Pitch(pitch)
      , m_Roll(roll)
      , m_UnwrapRes(unwrapRes)
    {
        BOB_ASSERT(agent.getOutputSize() == unwrapRes);
        BOB_ASSERT(units::math::fmod(360_deg, m_YawStep) == 0_deg);

        // This rotater doesn't support mask images
        BOB_ASSERT(maskImage.empty());
    }

    template<class Func>
    void rotate(Func func)
    {
        cv::Mat fr, mask;
        size_t i = 0;
        for (degree_t yaw = 0_deg; yaw < 360_deg; yaw += m_YawStep, i++) {
            m_Agent.setAttitude(yaw, m_Pitch, m_Roll);
            m_Agent.readGreyscaleFrame(fr);
            func(fr, mask, i);
        }
    }

    size_t numRotations() const
    {
        return 360_deg / m_YawStep;
    }

    units::angle::radian_t columnToHeading(size_t column) const
    {
        return units::angle::turn_t{ (double) column / (double) m_UnwrapRes.width };
    }

    template<typename... Ts>
    static auto
    create(Ts &&... args)
    {
        return AntWorldRotater(std::forward<Ts>(args)...);
    }

private:
    AntWorld::AntAgent &m_Agent;
    const degree_t m_YawStep, m_Pitch, m_Roll;
    const cv::Size m_UnwrapRes;
}; // AntWorldRotater
} // Navigation
} // BoBRobotics
