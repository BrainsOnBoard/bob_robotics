#pragma once

// Standard C includes
#include <cassert>

// Third-party includes
#include "../third_party/units.h"

// BoB robotics includes
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
        assert(agent.getOutputSize() == unwrapRes);

        // This rotater doesn't support mask images
        assert(maskImage.empty());

        loadImage();
    }

    cv::Mat &getImage()
    {
        return m_Frame;
    }

    cv::Mat getMaskImage()
    {
        return cv::Mat();
    }

    bool next()
    {
        m_CurrentYaw += m_YawStep;
        if (m_CurrentYaw >= 360_deg) {
            return false;
        }

        loadImage();
        return true;
    }

    size_t count() const
    {
        return m_CurrentYaw / m_YawStep;
    }

    size_t max() const
    {
        return 360_deg / m_YawStep;
    }

private:
    AntWorld::AntAgent &m_Agent;
    const degree_t m_YawStep, m_Pitch, m_Roll;
    degree_t m_CurrentYaw = 0_deg;
    cv::Mat m_Frame;

    void loadImage()
    {
        m_Agent.setAttitude(m_CurrentYaw, m_Pitch, m_Roll);
        m_Agent.readGreyscaleFrame(m_Frame);
    }
}; // AntWorldRotater
} // Navigation
} // BoBRobotics