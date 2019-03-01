#pragma once

// Third-party includes
#include "../third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <limits>
#include <utility>
#include <vector>

namespace BoBRobotics {
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
        , m_RobotVerticesPoints(robotDimensions.size())
        , m_XLower(inf())
        , m_YLower(inf())
    {
        // If there are no obstacles, then our job is easy
        if (objects.size() == 0) {
            return;
        }

        // Calculate vertices of objects after resizing to take into account buffer
        m_ResizedObjects.reserve(objects.size());
        for (auto &object : objects) {
            auto matrix = vectorToEigen(object);

            // Centre the object on the origin
            const Eigen::Vector2d centre = matrix.colwise().mean();
            Eigen::Matrix<double, 2, 2> translation;
            matrix.col(0).array() -= centre[0];
            matrix.col(1).array() -= centre[1];

            // Scale the object so we figure out the buffer zone around it
            const double width = matrix.col(0).maxCoeff() - matrix.col(0).minCoeff();
            const double scale = 1.0 + (bufferSize.value() / width);
            Eigen::Matrix<double, 2, 2> scaleMatrix;
            scaleMatrix << scale, 0,
                           0, scale;
            matrix *= scale;

            // Translate the object back to its origin location
            matrix.col(0).array() += centre[0];
            matrix.col(1).array() += centre[1];

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
        for (auto &object : m_ResizedObjects) {
            points.resize(object.rows());
            eigenToPoints(points, object);
            fillConvexPoly(m_ObjectsMap, points, cv::Scalar{ 0xff });
        }
    }

    const auto &getResizedObjects() const
    {
        return m_ResizedObjects;
    }

    template<class PoseType>
    void setRobotPose(const PoseType &pose)
    {
        // Rotate coords
        using namespace units::math;
        const double sinth = sin(pose.yaw()), costh = cos(pose.yaw());
        Eigen::Matrix<double, 2, 2> rotationMatrix;
        rotationMatrix << costh, -sinth,
                          sinth, costh;
        m_RobotVertices = m_RobotDimensions * rotationMatrix;

        // Translate
        m_RobotVertices.col(0).array() += static_cast<meter_t>(pose.x()).value();
        m_RobotVertices.col(1).array() += static_cast<meter_t>(pose.y()).value();
    }

    const auto &getRobotVertices() const
    {
        return m_RobotVertices;
    }

    bool collisionOccurred() const
    {
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
                return true;
            }
        }

        // No collision
        return false;
    }

private:
    const meter_t m_GridSize;
    const Eigen::MatrixX2d m_RobotDimensions;
    Eigen::MatrixX2d m_RobotVertices;
    mutable std::vector<cv::Point2i> m_RobotVerticesPoints;
    meter_t m_XLower, m_YLower;
    cv::Mat m_ObjectsMap;
    mutable cv::Mat m_RobotMap;
    std::vector<Eigen::MatrixX2d> m_ResizedObjects;

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
        return meter_t{ std::numeric_limits<double>::infinity() };
    }

}; // CollisionDetector
} // BoBRobotics
