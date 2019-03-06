#pragma once

// BoB robotics includes
#include "collision.h"
#include "geometry.h"
#include "pose.h"
#include "../robots/tank.h"
#include "../robots/tank_pid.h"

// Third-party includes
#include "../third_party/units.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <list>
#include <vector>

namespace BoBRobotics {
using namespace units::literals;

class ObstacleCircumnavigator {
    using meter_t = units::length::meter_t;
    using radian_t = units::angle::radian_t;

public:
    enum class State {
        DoingNothing,
        StartingCircumnavigation,
        Circumnavigating
    };

    ObstacleCircumnavigator(Robots::Tank &robot,
                            CollisionDetector &collisionDetector,
                            meter_t stoppingDistance = 3_cm,
                            float kp = 0.1f,
                            float ki = 0.1f,
                            float kd = 0.1f,
                            float averageSpeed = 0.5f)
        : m_TankPID(robot, kp, ki, kd, stoppingDistance, 3_deg, 45_deg, averageSpeed)
        , m_CollisionDetector(collisionDetector)
    {}

    void update(const Pose2<meter_t, radian_t> &robotPose)
    {
        if (m_PIDWaypoints.empty()) {
            // ... not following route
            m_CollisionDetector.setRobotPose(robotPose);
            if (m_CollisionDetector.collisionOccurred()) {
                m_TankPID.getRobot().stopMoving();
                startCircumnavigating(robotPose);
            }
        } else {
            updatePID(robotPose);
        }
    }

    auto getState() const { return m_State; }

    const auto &getPIDWaypoints() const { return m_PIDWaypoints; }

private:
    Robots::TankPID m_TankPID;
    EigenSTDVector<Eigen::Matrix2d> m_ObjectLines;
    std::list<Vector2<meter_t>> m_PIDWaypoints;
    Eigen::MatrixX2d m_ObjectPerimeter;
    State m_State = State::DoingNothing;
    CollisionDetector &m_CollisionDetector;

    void startCircumnavigating(const Pose2<meter_t, radian_t> &robotPose)
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
        Eigen::Vector2d robotGoal;
        robotGoal << 100 * cos(robotPose.yaw()), 100 * sin(robotPose.yaw());
        auto distToGoal = std::numeric_limits<double>::infinity();
        Eigen::Vector2d leavePoint;
        int whichLeaveLine = -1;

        // **TODO**: Change this to use an infinite straight line -- this is a bit gross
        Eigen::Matrix2d robotLine;
        robotLine(0, 0) = robotPose.x().value();
        robotLine(0, 1) = robotPose.y().value();
        robotLine(1, 0) = robotPose.x().value() + 10 * cos(robotPose.yaw()).value();
        robotLine(1, 1) = robotPose.x().value() + 10 * sin(robotPose.yaw()).value();

        // Get perimeter around object, resize it for calculating route
        m_ObjectPerimeter = objectVerts;
        resizePolygonBy(m_ObjectPerimeter, m_TankPID.getRobot().getRobotWidth());
        EigenSTDVector<Eigen::Matrix2d> perimeterLines(m_ObjectPerimeter.size());
        polygonToLines(perimeterLines, m_ObjectPerimeter);

        // Take the nearest point on the opposite side of the perimeter as the goal
        for (int i = 0; i < m_ObjectPerimeter.rows(); i++) {
            Eigen::Vector2d point;
            if (calculateIntersection(point, perimeterLines[i], robotLine)) {
                const auto dx = point(0) - robotGoal(0);
                const auto dy = point(1) - robotGoal(1);
                const auto dist = hypot(dx, dy);
                if (dist < distToGoal) {
                    whichLeaveLine = i;
                    leavePoint = point;
                    distToGoal = dist;
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
            const int lineMax = ((whichLine < whichLeaveLine) ? whichLeaveLine : (whichLeaveLine + m_ObjectPerimeter.rows()));
            for (int i = whichLine + 1; i <= lineMax; i++) {
                const Eigen::Vector2d next = m_ObjectPerimeter.row(i % m_ObjectPerimeter.rows());
                m_PIDWaypoints.emplace_back(meter_t{ next.x() }, meter_t{ next.y() });
            }
            m_PIDWaypoints.emplace_back(meter_t{ leavePoint.x() }, meter_t{ leavePoint.y() });

            // Start PID control of robot
            std::cout << "Driving to " << m_PIDWaypoints.front() << std::endl;
            m_TankPID.start(m_PIDWaypoints.front());
            m_State = State::StartingCircumnavigation;
        }
    }

    void updatePID(const Pose2<meter_t, radian_t> &robotPose)
    {
        m_State = State::Circumnavigating;

        if (m_TankPID.driveRobot(robotPose)) {
            // ... then we've reached a waypoint
            m_PIDWaypoints.pop_front();

            // If there are more waypoints, head for the next one
            if (m_PIDWaypoints.empty()) {
                m_State = State::DoingNothing;
            } else {
                std::cout << "Driving to " << m_PIDWaypoints.front() << std::endl;
                m_TankPID.start(m_PIDWaypoints.front());
            }
        }
    }
};
} // BoBRobotics
