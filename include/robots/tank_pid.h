#pragma once

// BoB robotics includes
#include "common/fsm.h"
#include "common/pose.h"
#include "common/stopwatch.h"
#include "tank.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <limits>

namespace BoBRobotics {
namespace Robots {

using namespace units::literals;

enum class TankPIDState
{
    Invalid,
    OrientingToGoal,
    DrivingToGoal,
    AtGoal
};

class TankPID
  : public FSM<TankPIDState>::StateHandler
{
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;
    using second_t = units::time::second_t;
    using Event = FSM<TankPIDState>::StateHandler::Event;

public:
    TankPID(Tank &robot, float Kp, float Ki, float Kd,
            meter_t distanceTolerance = 5_cm,
            radian_t angleTolerance = 3_deg,
            radian_t startTurningThreshold = 45_deg,
            float averageSpeed = 0.5f);

    void start(const Vector2<meter_t> &goal);

    bool driveRobot(const Pose2<meter_t, radian_t> &robotPose);

    virtual bool handleEvent(TankPIDState state, Event event) override;

private:
    FSM<TankPIDState> m_StateMachine;
    Tank &m_Robot;
    Pose2<meter_t, radian_t> m_RobotPose;
    Vector2<meter_t> m_Goal;
    const meter_t m_DistanceTolerance;
    const radian_t m_AngleTolerance, m_StartTurningThreshold;
    radian_t m_HeadingOffset;
    double m_LastError = std::numeric_limits<double>::quiet_NaN();
    Stopwatch m_Stopwatch;
    const float m_Kp, m_Ki, m_Kd, m_AverageSpeed;
}; // TankPID
} // Robots
} // BoBRobotics