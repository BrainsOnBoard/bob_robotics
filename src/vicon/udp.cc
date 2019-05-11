// BoB robotics includes
#include "common/circstat.h"
#include "vicon/udp.h"

namespace BoBRobotics
{
namespace Vicon
{
using namespace units::literals;

void
ObjectData::update(uint32_t frameNumber,
                   Pose3<millimeter_t, radian_t> pose)
{
    // Log the time when this packet was received
    m_ReceivedTimer.start();

    // Cache frame number
    m_FrameNumber = frameNumber;

    // Copy vectors into class
    m_Pose = pose;
}

uint32_t
ObjectData::getFrameNumber() const
{
    return m_FrameNumber;
}

const std::string &
ObjectData::getName() const
{
    return m_Name;
}

Stopwatch::Duration
ObjectData::timeSinceReceived() const
{
    return m_ReceivedTimer.elapsed();
}

void
ObjectDataVelocity::update(uint32_t frameNumber,
                           Pose3<millimeter_t, radian_t> pose)
{
    constexpr second_t frameS = 10_ms;
    constexpr second_t smoothingS = 30_ms;

    // Calculate time since last frame
    const uint32_t deltaFrames = frameNumber - getFrameNumber();
    const auto deltaS = frameS * deltaFrames;

    // Calculate exponential smoothing factor
    const double alpha = 1.0 - units::math::exp(-deltaS / smoothingS);

    // Calculate velocities (m/s)
    const auto oldPosition = getPosition<>();
    calculateVelocities(pose.position(), oldPosition, m_Velocity, deltaS, alpha, std::minus<millimeter_t>());

    // Calculate angular velocities (rad/s)
    const auto oldAttitude = getAttitude<>();
    const auto circDist = [](auto angle1, auto angle2) {
        return circularDistance(angle1, angle2);
    };
    calculateVelocities(pose.attitude(), oldAttitude, m_AngularVelocity, deltaS, alpha, circDist);

    // Superclass
    // **NOTE** this is at the bottom so OLD position can be accessed
    ObjectData::update(frameNumber, pose);
}

TimedOutError::TimedOutError()
  : std::runtime_error("Timed out waiting for Vicon data")
{}

} // namespace Vicon
} // BoBRobotics
