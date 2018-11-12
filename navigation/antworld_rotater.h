#pragma once

// Third-party includes
#include "../third_party/units.h"

// BoB robotics includes
#include "../common/assert.h"
#include "../libantworld/agent.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
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
    {
        BOB_ASSERT(agent.getOutputSize() == unwrapRes);

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

    size_t max() const
    {
        return 360_deg / m_YawStep;
    }

private:
    AntWorld::AntAgent &m_Agent;
    const degree_t m_YawStep, m_Pitch, m_Roll;
}; // AntWorldRotater
} // Navigation
} // BoBRobotics
