#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/geometry.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <limits>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Robots {
using namespace units::literals;

class CollisionDetector {
    using meter_t = units::length::meter_t;
    using Limits = std::pair<meter_t, meter_t>;

public:
    template<class RobotVertices, class ObjectVertices>
    CollisionDetector(const RobotVertices &robotDimensions,
                      const std::vector<ObjectVertices> &objects,
                      meter_t bufferSize = 30_cm,
                      meter_t gridSize = 1_cm)
        : m_GridSize(gridSize)
        , m_RobotDimensions(vectorToEigen(robotDimensions))
        , m_RobotVertices(m_RobotDimensions)
        , m_RobotVerticesPoints(robotDimensions.size())
        , m_XLower(inf())
        , m_YLower(inf())
    {
        // If there are no obstacles, then our job is easy
        if (objects.size() == 0) {
            return;
        }

        // Maximum number of supported objects
        BOB_ASSERT(objects.size() <= 254);

        // Calculate vertices of objects after resizing to take into account buffer
        m_ResizedObjects.reserve(objects.size());
        for (auto &object : objects) {
            auto matrix = vectorToEigen(object);

            // Scale the object to include a "buffer" around it
            resizePolygonBy(matrix, bufferSize);

            // Store for later
            m_ResizedObjects.emplace_back(std::move(matrix));
        }

        // Calculate x and y limits
        meter_t xUpper = -inf(), yUpper = -inf();
        for (auto &object : m_ResizedObjects) {
            const auto xmin = object.col(0).minCoeff();
            const auto xmax = object.col(0).maxCoeff();
            const auto ymin = object.col(1).minCoeff();
            const auto ymax = object.col(1).maxCoeff();
            if (xmin < m_XLower.value()) {
                m_XLower = meter_t{ xmin };
            }
            if (xmax > xUpper.value()) {
                xUpper = meter_t{ xmax };
            }
            if (ymin < m_YLower.value()) {
                m_YLower = meter_t{ ymin };
            }
            if (ymax > yUpper.value()) {
                yUpper = meter_t{ ymax };
            }
        }

        // Create maps to draw objects and robot on
        const auto toPixels = [&](meter_t lower, meter_t upper) {
            return static_cast<int>((upper - lower) / gridSize);
        };
        m_ObjectsMap.create(toPixels(m_YLower, yUpper), toPixels(m_XLower, xUpper), CV_8UC1);
        m_ObjectsMap = cv::Scalar{ 0 }; // Fill with zeroes
        m_RobotMap.create(m_ObjectsMap.size(), CV_8UC1);

        // Draw each of the objects as a filled polygon on m_ObjectsMap
        std::vector<cv::Point2i> points;
        for (size_t i = 0; i < m_ResizedObjects.size(); i++) {
            points.resize(m_ResizedObjects[i].rows());
            eigenToPoints(points, m_ResizedObjects[i]);
            fillConvexPoly(m_ObjectsMap, points, cv::Scalar{ static_cast<double>(i + 1) });
        }
    }

    template<class PoseType>
    void setRobotPose(const PoseType &pose)
    {
        // Rotate coords
        using namespace units::math;
        const double sinth = sin(pose.yaw()), costh = cos(pose.yaw());
        Eigen::Matrix2d rotationMatrix;
        rotationMatrix << costh, -sinth,
                          sinth, costh;
        m_RobotVertices = m_RobotDimensions * rotationMatrix;

        // Translate
        m_RobotVertices.col(0).array() += static_cast<meter_t>(pose.x()).value();
        m_RobotVertices.col(1).array() += static_cast<meter_t>(pose.y()).value();
    }

    template<class PoseType>
    bool wouldCollide(const PoseType &pose)
    {
        setRobotPose(pose);
        return collisionOccurred();
    }

    const EigenSTDVector<Eigen::MatrixX2d> &getResizedObjects() const;
    const Eigen::MatrixX2d &getRobotVertices() const;
    bool collisionOccurred();

    size_t getCollidedObjectId() const;

private:
    const meter_t m_GridSize;
    const Eigen::MatrixX2d m_RobotDimensions;
    Eigen::MatrixX2d m_RobotVertices;
    std::vector<cv::Point2i> m_RobotVerticesPoints;
    meter_t m_XLower, m_YLower;
    cv::Mat m_ObjectsMap;
    cv::Mat m_RobotMap;
    std::vector<Eigen::MatrixX2d, Eigen::aligned_allocator<Eigen::MatrixX2d>> m_ResizedObjects;
    size_t m_CollidedObjectId = std::numeric_limits<size_t>::max();

    //! Convert Eigen Matrix to OpenCV pionts
    template<class MatrixType, class PointsArray>
    void eigenToPoints(PointsArray &points, const MatrixType &matrix) const
    {
        for (int i = 0; i < matrix.rows(); i++) {
            points[i] = cv::Point2i{ static_cast<int>((meter_t{ matrix(i, 0) } - m_XLower) / m_GridSize),
                                     static_cast<int>((meter_t{ matrix(i, 1) } - m_YLower) / m_GridSize) };
        }
    }

    //! Convert from our pose types to an Eigen Matrix
    template<class VectorArray>
    static auto vectorToEigen(const VectorArray &vectors)
    {
        Eigen::MatrixX2d matrix(vectors.size(), 2);
        for (size_t i = 0; i < vectors.size(); i++) {
            matrix(i, 0) = static_cast<meter_t>(vectors[i].x()).value();
            matrix(i, 1) = static_cast<meter_t>(vectors[i].y()).value();
        }
        return matrix;
    }

    static constexpr meter_t inf()
    {
        return meter_t{ std::numeric_limits<UNIT_LIB_DEFAULT_TYPE>::infinity() };
    }

}; // CollisionDetector
} // Robots
} // BoBRobotics
