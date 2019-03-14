#pragma once

// BoB robotics includes
#include "assert.h"

// Third-party includes
#include "../third_party/units.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <limits>
#include <vector>

template<class T>
using EigenSTDVector = std::vector<T, Eigen::aligned_allocator<T>>;

namespace BoBRobotics {

template<int Rows>
inline void
resizePolygonBy(Eigen::Matrix<double, Rows, 2> &polygon, units::length::meter_t extraSize)
{
    BOB_ASSERT(polygon.rows() > 2); // Check it's a polygon
    const auto cols = polygon.colwise();

    // Centre the object on the origin
    const Eigen::Vector2d centre = cols.mean();
    Eigen::Matrix2d translation;
    polygon.col(0).array() -= centre[0];
    polygon.col(1).array() -= centre[1];

    // Scale the object so we figure out the buffer zone around it
    const Eigen::Array2d width = cols.maxCoeff() - cols.minCoeff();
    const auto scale = 1.0 + (2.0 * extraSize.value()) / width;
    polygon.col(0).array() *= scale(0);
    polygon.col(1).array() *= scale(1);

    // Translate the object back to its original location
    polygon.col(0).array() += centre[0];
    polygon.col(1).array() += centre[1];
}

template<int Rows>
inline void
polygonToLines(EigenSTDVector<Eigen::Matrix2d> &lines, const Eigen::Matrix<double, Rows, 2> &polygon)
{
    BOB_ASSERT(polygon.rows() > 2); // Check it's a polygon

    lines.clear();
    for (int i = 0; i < polygon.rows() - 1; i++) {
        lines.emplace_back(2, 2);
        lines.back() << polygon(i, 0), polygon(i, 1),
                polygon(i + 1, 0), polygon(i + 1, 1);
    }
    lines.emplace_back(2, 2);
    const long max = polygon.rows() - 1;
    lines.back() << polygon(max, 0), polygon(max, 1),
                    polygon(0, 0), polygon(0, 1);
}

inline bool
calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1, const Eigen::Matrix2d &line2)
{
    const auto a1 = line1(1, 1) - line1(0, 1);
    const auto b1 = line1(0, 0) - line1(1, 0);
    const auto c1 = a1 * line1(0, 0) + b1 * line1(0, 1);
    const auto a2 = line2(1, 1) - line2(0, 1);
    const auto b2 = line2(0, 0) - line2(1, 0);
    const auto c2 = a2 * line2(0, 0) + b2 * line2(0, 1);

    const auto det = a1 * b2 - a2 * b1;
    if (det == 0.0) {
        // Lines are parallel
        return false;
    }

    point << (b2 * c1 - b1 * c2) / det,
             (a1 * c2 - a2 * c1) / det;

    const auto min = [](const auto &line, int index) {
        return line.col(index).minCoeff();
    };
    const auto max = [](const auto &line, int index) {
        return line.col(index).maxCoeff();
    };

    // Margin of error for floating-point numbers
    constexpr double tol = std::numeric_limits<double>::epsilon();

    // Check that the point is on line 1
    const bool a = min(line1, 0) <= point(0) + tol;
    const bool b = max(line1, 0) >= point(0) - tol;
    const bool c = min(line1, 1) <= point(1) + tol;
    const bool d = max(line1, 1) >= point(1) - tol;

    // Check that the point is on line 2
    const bool e = min(line2, 0) <= point(0) + tol;
    const bool f = max(line2, 0) >= point(0) - tol;
    const bool g = min(line2, 1) <= point(1) + tol;
    const bool h = max(line2, 1) >= point(1) - tol;

    return a && b && c && d && e && f && g && h;
}
} // BoBRobotics
