// BoB robotics includes
#include "navigation/antworld_rotater.h"
#include "common/assert.h"

using namespace units::angle;

namespace BoBRobotics {
namespace Navigation {

AntWorldRotater::AntWorldRotater(const cv::Size &unwrapRes,
                                 const cv::Mat &maskImage,
                                 AntWorld::AntAgent &agent,
                                 const degree_t yawStep,
                                 const degree_t pitch,
                                 const degree_t roll)
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

size_t
AntWorldRotater::numRotations() const
{
    return 360_deg / m_YawStep;
}

units::angle::radian_t
AntWorldRotater::columnToHeading(size_t column) const
{
    return units::angle::turn_t{ (double) column / (double) m_UnwrapRes.width };
}

} // Navigation
} // BoBRobotics
