#pragma once

// BoB robotics includes
#include "macros.h"
#include "circstat.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
template<class T>
using EigenSTDVector = std::vector<T, Eigen::aligned_allocator<T>>;

template<class T = double>
constexpr inline T inf()
{
    return std::numeric_limits<T>::infinity();
}

using Line2 = Eigen::Hyperplane<double, 2>;

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

double
distance2D(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

double min(const Eigen::Matrix2d &line, int index);
double max(const Eigen::Matrix2d &line, int index);

bool calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1Points, const Line2 &line2);
bool calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1Points, const Eigen::Matrix2d &line2Points);
} // BoBRobotics
