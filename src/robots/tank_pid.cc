// BoB robotics includes
#include "common/circstat.h"
#include "common/logging.h"
#include "robots/tank_pid.h"

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>

using namespace units::angle;
using namespace units::length;

namespace BoBRobotics {
namespace Robots {

TankPID::TankPID(Tank &robot,
                 float Kp,
                 float Ki,
                 float Kd,
                 meter_t distanceTolerance,
                 radian_t angleTolerance,
                 radian_t startTurningThreshold,
                 float averageSpeed)
    : m_StateMachine(this, TankPIDState::Invalid)
    , m_Robot(robot)
    , m_DistanceTolerance(distanceTolerance)
    , m_AngleTolerance(angleTolerance)
    , m_StartTurningThreshold(startTurningThreshold)
    , m_Kp(Kp)
    , m_Ki(Ki)
    , m_Kd(Kd)
    , m_AverageSpeed(averageSpeed)
{}

void TankPID::start(const Vector2<meter_t> &goal)
{
    m_Goal = goal;

    /*
        * We start off assuming that the heading is way off and that we need to
        * turn on the spot.
        */
    m_StateMachine.transition(TankPIDState::OrientingToGoal);
}

bool TankPID::driveRobot(const Pose2<meter_t, radian_t> &robotPose)
{
    m_RobotPose = robotPose;
    const radian_t headingToGoal = units::math::atan2(m_Goal.y() - m_RobotPose.y(),
                                                        m_Goal.x() - m_RobotPose.x());
    m_HeadingOffset = circularDistance(headingToGoal, m_RobotPose.yaw());

    // Get state machine to carry out appropriate action
    m_StateMachine.update();

    // Return true if we're at goal (within m_DistanceTolerance)
    return m_StateMachine.getCurrentState() == TankPIDState::AtGoal;
}

bool TankPID::handleEvent(TankPIDState state, Event event)
{
    using namespace units::math;

    if (event == Event::Update && m_Goal.distance2D(m_RobotPose) < m_DistanceTolerance) {
        m_StateMachine.transition(TankPIDState::AtGoal);
        return true;
    }

    switch (state) {
    case TankPIDState::OrientingToGoal:
        if (event == Event::Enter) {
            LOG_DEBUG << "Starting turning";
        } else if (event == Event::Update) {
            /*
                * If m_HeadingOffset is suitably small, we've finished turning
                * towards the goal.
                */
            if (abs(m_HeadingOffset) <= m_AngleTolerance) {
                // Start driving to the goal in a straight(ish) line
                m_StateMachine.transition(TankPIDState::DrivingToGoal);
            } else if (m_HeadingOffset < 0_deg) {
                m_Robot.tank(1.f, -1.f);
            } else {
                m_Robot.tank(-1.f, 1.f);
            }
        } else { // Exit
            m_Robot.stopMoving();
            LOG_DEBUG << "Stopping turning";
        }
        break;
    case TankPIDState::DrivingToGoal: {
        // Error value used in PID control
        const float error = m_HeadingOffset.value();

        if (event == Event::Enter) {
            // Start timing
            m_Stopwatch.start();
        } else if (event == Event::Update) {
            // If the heading offset is big, then start turning on the spot
            if (abs(m_HeadingOffset) > m_StartTurningThreshold) {
                m_StateMachine.transition(TankPIDState::OrientingToGoal);
                return true;
            }

            const float dt = static_cast<second_t>(m_Stopwatch.lap()).value();

            // PID controller for straight(ish) portion of course
            const float p = m_Kp * error;
            const float i = m_Ki * (error + m_LastError * dt);
            const float d = m_Kd * (m_LastError - error) / dt;

            const float differential = p + i + d;

            const float v1 = std::min(1.f, std::max(0.f, m_AverageSpeed + differential));
            const float v2 = std::min(1.f, std::max(0.f, m_AverageSpeed - differential));
            if (differential >= 0.f) {
                m_Robot.tank(v1, v2);
            } else {
                m_Robot.tank(v2, v1);
            }
        } else { // Exit
            m_Robot.stopMoving();
            return true;
        }

        // Keep track of previous error for I and D terms of PID
        m_LastError = error;
    } break;
    case TankPIDState::AtGoal:
        if (event == Event::Enter) {
            LOG_INFO << "Reached goal";
        }
        break;
    case TankPIDState::Invalid:
        break;
    }

    return true;
}

} // Robots
} // BoBRobotics