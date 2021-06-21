// BoB robotics includes
#include "robots/control/collision_detector.h"

namespace BoBRobotics {
namespace Robots {

const EigenSTDVector<Eigen::MatrixX2d> &
CollisionDetector::getResizedObjects() const
{
    return m_ResizedObjects;
}

const Eigen::MatrixX2d &
CollisionDetector::getRobotVertices() const
{
    return m_RobotVertices;
}

bool
CollisionDetector::collisionOccurred()
{
    // We don't care about the position, so don't do anything with it
    Vector2<meter_t> pos;
    return collisionOccurred(pos);
}

bool
CollisionDetector::collisionOccurred(Vector2<meter_t> &firstCollisionPosition)
{
    // Clear this value
    m_CollidedObjectId = std::numeric_limits<size_t>::max();

    // If there aren't obstacles, we can't have hit them
    if (m_ResizedObjects.size() == 0) {
        return false;
    }

    // Fill map with zeroes
    m_RobotMap = cv::Scalar{ 0 };

    // Draw agent onto map
    eigenToPoints(m_RobotVerticesPoints, m_RobotVertices);
    fillConvexPoly(m_RobotMap, m_RobotVerticesPoints, cv::Scalar{ 0xff });

    // Check for collision
    for (int i = 0; i < m_RobotMap.size().area(); i++) {
        if (m_ObjectsMap.data[i] & m_RobotMap.data[i]) {
            // Calculate collision point
            const auto coordPixel = div(i, m_RobotMap.cols);
            firstCollisionPosition.x() = m_XLower + (coordPixel.rem * m_GridSize);
            firstCollisionPosition.y() = m_YLower + (coordPixel.quot * m_GridSize);

            m_CollidedObjectId = m_ObjectsMap.data[i] - 1;
            return true;
        }
    }

    // No collision
    return false;
}

size_t
CollisionDetector::getCollidedObjectId() const
{
    return m_CollidedObjectId;
}

} // Robots
} // BoBRobotics
