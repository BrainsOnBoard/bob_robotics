#pragma once

// BoB robotics includes
#include "../common/timer.h"
#include "../libantworld/agent.h"

// Third-party includes
#include "../third_party/units.h"

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
using namespace units::length;
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::AntWorldRotater
//------------------------------------------------------------------------
class AntWorldRotater
{
public:
    AntWorldRotater(const cv::Size &, AntWorld::AntAgent &agent, const degree_t yawStep = 1_deg, const degree_t pitch = 0_deg, const degree_t roll = 0_deg)
      : m_Agent(agent)
      , m_YawStep(yawStep)
      , m_Pitch(pitch)
      , m_Roll(roll)
    {
        loadImage();
    }

    cv::Mat &getImage()
    {
        return m_Frame;
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