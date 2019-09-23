// BoB robotics includes
#include "antworld/tank.h"
#include "common/path.h"
#include "common/logging.h"

using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace AntWorld {
Tank::Tank(const cv::Size &renderSize,
           meters_per_second_t maxVelocity,
           meter_t agentHeight)
    : SimulatedTank<meter_t, degree_t>(maxVelocity)
    , m_Window(AntWorld::Camera::initialiseWindow(renderSize))
    , m_Renderer(256, 0.001, 1000.0, 360_deg)
    , m_Camera(*m_Window, m_Renderer, renderSize)
    , m_AgentHeight(agentHeight)
{
    m_Renderer.getWorld().load(Path::getResourcesPath() / "antworld" / "world5000_gray.bin",
                               {0.0f, 1.0f, 0.0f}, {0.898f, 0.718f, 0.353f});

    // Position in centre of world
    const auto &max = m_Renderer.getWorld().getMaxBound();
    const auto &min = m_Renderer.getWorld().getMinBound();
    Pose2<meter_t, degree_t> initialPose;
    initialPose.x() = (max.x() - min.x()) / 2;
    initialPose.y() = (max.y() - min.y()) / 2;
    initialPose.z() = m_AgentHeight;
    setPose(initialPose);
}

std::unique_ptr<Video::Input>
Tank::getCamera()
{
    return std::make_unique<Tank::Camera>(*this);
}

void
Tank::updatePose()
{
    SimulatedTank<meter_t, degree_t>::doUpdatePose();
    const auto &pose = SimulatedTank<meter_t, degree_t>::getPoseRaw();
    m_Camera.setPosition(pose.x(), pose.y(), m_AgentHeight);
    m_Camera.setAttitude(pose.yaw(), 0_deg, 0_deg);
}

sf::Window &
Tank::getWindow()
{
    return *m_Window;
}

Tank::Camera::Camera(Tank &tank)
  : m_Tank(tank)
{}

bool
Tank::Camera::readFrame(cv::Mat &fr)
{
    m_Tank.updatePose();
    return m_Tank.m_Camera.readFrame(fr);
}

void
Tank::setPose(const Pose2<meter_t, degree_t> &pose)
{
    SimulatedTank<meter_t, degree_t>::setPose(pose);
}

cv::Size
Tank::Camera::getOutputSize() const
{
    return m_Tank.m_Camera.getOutputSize();
}

bool
Tank::Camera::needsUnwrapping() const
{
    return false;
}

const Vector3<meter_t> &
Tank::getMinBound()
{
    return m_Renderer.getWorld().getMinBound();
}

const Vector3<meter_t> &
Tank::getMaxBound()
{
    return m_Renderer.getWorld().getMaxBound();
}

} // AntWorld
} // BoBRobotics
