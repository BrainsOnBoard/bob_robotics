#pragma once

// BoB robotics includes
#include "common/geometry.h"
#include "common/pose.h"
#include "robots/control/collision_detector.h"
#include "robots/control/tank_pid.h"
#include "robots/control/positioner.h"
#include "robots/tank.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <list>
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

// Forward declarations
template<class PoseGetterType>
class ObstacleCircumnavigator;
template<class PositionerType, class PoseGetterType>
class ObstacleAvoidingPositioner;

template<class PoseGetterType, class... Args>
auto createObstacleCircumnavigator(Robots::Tank &tank,
                                   PoseGetterType &poseGetter,
                                   Args&&... otherArgs)
{
    return ObstacleCircumnavigator<PoseGetterType>{ tank, poseGetter, std::forward<Args>(otherArgs)... };
}

template<class PositionerType, class PoseGetterType>
auto createObstacleAvoidingPositioner(PositionerType &positioner,
                                      ObstacleCircumnavigator<PoseGetterType> &circumnavigator)
{
    return ObstacleAvoidingPositioner<PositionerType, PoseGetterType>{ positioner, circumnavigator };
}

enum class ObstacleCircumnavigatorState {
    DoingNothing,
    StartingCircumnavigation,
    Circumnavigating
};

template<class PositionerType, class PoseGetterType>
class ObstacleAvoidingPositioner
  : public Robots::PositionerBase<ObstacleAvoidingPositioner<PositionerType, PoseGetterType>>
{
public:
    ObstacleAvoidingPositioner(PositionerType &positioner,
                               ObstacleCircumnavigator<PoseGetterType> &circumnavigator)
      : m_Positioner(positioner)
      , m_Circumnavigator(circumnavigator)
    {}

    void reset()
    {
        m_Positioner.reset();
    }

    const auto &getPose() const
    {
        return m_Positioner.getPose();
    }

    const auto &getRobot() const { return m_Positioner.getRobot(); }
    auto &getRobot() { return m_Positioner.getRobot(); }

    template<class PoseType>
    void moveTo(const PoseType &pose)
    {
        BOB_ASSERT(!m_Circumnavigator.wouldCollide(pose));
        m_Positioner.moveTo(pose);
    }

    bool pollPositioner()
    {
        m_Circumnavigator.update();
        if (m_Circumnavigator.getState() == ObstacleCircumnavigatorState::DoingNothing) {
            return m_Positioner.pollPositioner();
        } else {
            return true;
        }
    }

private:
    PositionerType &m_Positioner;
    ObstacleCircumnavigator<PoseGetterType> &m_Circumnavigator;
}; // ObstacleAvoidingPositioner

template<class PoseGetterType>
class ObstacleCircumnavigator {
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;

public:
    using State = ObstacleCircumnavigatorState;

    ObstacleCircumnavigator(Robots::Tank &robot,
                            PoseGetterType &poseGetter,
                            Robots::CollisionDetector &collisionDetector,
                            meter_t stoppingDistance = 3_cm,
                            float kp = 0.1f,
                            float ki = 0.1f,
                            float kd = 0.1f,
                            float averageSpeed = 0.5f)
        : m_TankPID(robot, poseGetter, kp, ki, kd, stoppingDistance, 3_deg, 45_deg, averageSpeed)
        , m_PoseGetter(poseGetter)
        , m_CollisionDetector(collisionDetector)
    {}

    template<class VectorType = Vector2<meter_t>>
    void update(const VectorType &robotGoal = Vector2<meter_t>::nan())
    {
        if (m_PIDWaypoints.empty()) {
            const auto pose = m_PoseGetter.getPose();

            // ... not following route
            m_CollisionDetector.setRobotPose(pose);
            if (m_CollisionDetector.collisionOccurred()) {
                m_TankPID.getRobot().stopMoving();
                startCircumnavigating(pose, robotGoal);
            }
        } else {
            updatePID();
        }
    }

    template<class PoseType>
    bool wouldCollide(const PoseType &pose) { return m_CollisionDetector.wouldCollide(pose); }

    auto getState() const { return m_State; }

    const auto &getPIDWaypoints() const { return m_PIDWaypoints; }

private:
    Robots::TankPID<PoseGetterType> m_TankPID;
    PoseGetterType &m_PoseGetter;
    EigenSTDVector<Eigen::Matrix2d> m_ObjectLines;
    std::list<Vector2<meter_t>> m_PIDWaypoints;
    Eigen::MatrixX2d m_ObjectPerimeter;
    State m_State = State::DoingNothing;
    Robots::CollisionDetector &m_CollisionDetector;

    void startCircumnavigating(const Pose2<meter_t, radian_t> &robotPose,
                               const Vector2<meter_t> &robotGoal)
    {
        using namespace units::math;

        // Get the vertices of the object we've (nearly) hit
        size_t objectId = m_CollisionDetector.getCollidedObjectId();
        const auto &objectVerts = m_CollisionDetector.getResizedObjects()[objectId];
        Eigen::Vector2d objectCentre;
        objectCentre << objectVerts.col(0).mean(), objectVerts.col(1).mean();

        // Convert to vector of lines
        m_ObjectLines.clear();
        polygonToLines(m_ObjectLines, objectVerts);

        // Find which side of the object intersects the line from robot to object
        Eigen::Matrix2d robotToObject;
        robotToObject << robotPose.x().value(), robotPose.y().value(),
                objectCentre.x(), objectCentre.y();
        const auto pos = std::find_if(m_ObjectLines.cbegin(), m_ObjectLines.cend(),
            [&robotToObject](const auto &line)
            {
                Eigen::Vector2d point;
                return calculateIntersection(point, robotToObject, line);
            });

        // No intersections
        BOB_ASSERT(pos != m_ObjectLines.cend());
        const int whichLine = static_cast<int>(std::distance(m_ObjectLines.cbegin(), pos));

        // Calculate the robot's goal assuming it's trying to go straight through the object
        auto distFromRobot = 0.0;
        Eigen::Vector2d leavePoint;
        int whichLeaveLine = -1;

        const Eigen::Vector2d robotPosition = { robotPose.x().value(), robotPose.y().value() };
        StraightLine robotLine;
        if (std::isnan(robotGoal.x().value())) {
            // No goal specified: assume robot wants to go in a straight line
            robotLine.m = tan(robotPose.yaw()).value();
            robotLine.c = robotPose.y().value() - robotLine.m * robotPose.x().value();
        } else {
            // Aim for robot's actual goal
            const Eigen::Vector2d goal = { robotGoal.x().value(), robotGoal.y().value() };
            robotLine = StraightLine::fromPoints(robotPosition, goal);
        }

        // Get perimeter around object, resize it for calculating route
        m_ObjectPerimeter = objectVerts;
        resizePolygonBy(m_ObjectPerimeter, m_TankPID.getRobot().getRobotWidth());
        EigenSTDVector<Eigen::Matrix2d> perimeterLines(m_ObjectPerimeter.size());
        polygonToLines(perimeterLines, m_ObjectPerimeter);

        /*
         * Take our (proximate) goal as the furthest point along robotLine that
         * intersects with m_ObjectPerimeter.
         */
        const int numVerts = m_ObjectPerimeter.rows();
        for (int i = 0; i < numVerts; i++) {
            Eigen::Vector2d point;
            if (calculateIntersection(point, perimeterLines[i], robotLine)) {
                const auto dist = distance2D(point, robotPosition);
                if (dist > distFromRobot) {
                    whichLeaveLine = i;
                    leavePoint = point;
                    distFromRobot = dist;
                }
            }
        }

        if (whichLeaveLine == -1 || whichLeaveLine == whichLine) {
            std::cerr << "Bad line number: " << whichLeaveLine << std::endl
                      << "(This is normal e.g. if moving backwards)" << std::endl
                      << "Aborting" << std::endl;
            m_State = State::DoingNothing;
        } else {
            // Append the waypoints to avoidLine (for display) and goals (for homing)
            double distanceClockwise = 0.0, distanceAntiClockwise = 0.0;

            std::vector<int> incIndices, decIndices;
            int lineMaxInc = ((whichLine < whichLeaveLine) ? whichLeaveLine : (whichLeaveLine + numVerts));
            const int lineMin = (whichLine + 1) % numVerts;
            if ((lineMaxInc % numVerts) == lineMin) {
                lineMaxInc = lineMin;
            }
            for (int i = lineMin; i <= lineMaxInc; i++) {
                incIndices.push_back(i % numVerts);
            }
            const int lineMaxDec = (whichLeaveLine > whichLine) ? (whichLine + numVerts) : whichLine;
            for (int i = lineMaxDec; i > whichLeaveLine; i--) {
                decIndices.push_back(i % numVerts);
            }

            // Get total distance going one way around object
            Eigen::Vector2d last = robotPosition;
            for (auto i : incIndices) {
                const Eigen::Vector2d next = m_ObjectPerimeter.row(i % numVerts);
                distanceAntiClockwise += distance2D(last, next);
                last = next;
            }
            distanceAntiClockwise += distance2D(last, leavePoint);

            // Get total distance going the other way
            last = robotPosition;
            for (auto i : decIndices) {
                const Eigen::Vector2d next = m_ObjectPerimeter.row(i % numVerts);
                distanceClockwise += distance2D(last, next);
                last = next;
            }
            distanceClockwise += distance2D(last, leavePoint);

            // Pick the shortest path
            for (auto i : (distanceAntiClockwise < distanceClockwise) ? incIndices : decIndices) {
                const Eigen::Vector2d next = m_ObjectPerimeter.row(i % numVerts);
                m_PIDWaypoints.emplace_back(meter_t{ next.x() }, meter_t{ next.y() });
            }
            m_PIDWaypoints.emplace_back(meter_t{ leavePoint.x() }, meter_t{ leavePoint.y() });

            // Start PID control of robot
            std::cout << "Driving to " << m_PIDWaypoints.front() << std::endl;
            m_TankPID.moveTo(m_PIDWaypoints.front());
            m_State = State::StartingCircumnavigation;
        }
    }

    void updatePID()
    {
        m_State = State::Circumnavigating;

        if (!m_TankPID.pollPositioner()) {
            // ... then we've reached a waypoint
            m_PIDWaypoints.pop_front();

            // If there are more waypoints, head for the next one
            if (m_PIDWaypoints.empty()) {
                m_State = State::DoingNothing;
            } else {
                std::cout << "Driving to " << m_PIDWaypoints.front() << std::endl;
                m_TankPID.moveTo(m_PIDWaypoints.front());
            }
        }
    }
};
} // Robots
} // BoBRobotics
