// BoB robotics includes
#include "common/geometry.h"
#include "common/macros.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Geometry>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
double
distance2D(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    return hypot(p2.x() - p1.x(), p2.y() - p1.y());
}


double min(const Eigen::Matrix2d &line, int index)
{
    return line.col(index).minCoeff();
}

double max(const Eigen::Matrix2d &line, int index)
{
    return line.col(index).maxCoeff();
}

bool
calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1Points, const Line2 &line2)
{
    const auto lineEq1 = Line2::Through(line1Points.row(0), line1Points.row(1));
    point = lineEq1.intersection(line2);

    // Margin of error for floating-point numbers
    constexpr double tol = std::numeric_limits<double>::epsilon();

    // Check that the point is on line 1
    const bool a = min(line1Points, 0) <= point(0) + tol;
    const bool b = max(line1Points, 0) >= point(0) - tol;
    const bool c = min(line1Points, 1) <= point(1) + tol;
    const bool d = max(line1Points, 1) >= point(1) - tol;
    return a && b && c && d;
}

bool
calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1Points, const Eigen::Matrix2d &line2Points)
{
    const auto lineEq2 = Line2::Through(line2Points.row(0), line2Points.row(1));
    if (!calculateIntersection(point, line1Points, lineEq2)) {
        return false;
    }

    // Margin of error for floating-point numbers
    constexpr double tol = std::numeric_limits<double>::epsilon();

    // Check that the point is on line 2
    const bool e = min(line2Points, 0) <= point(0) + tol;
    const bool f = max(line2Points, 0) >= point(0) - tol;
    const bool g = min(line2Points, 1) <= point(1) + tol;
    const bool h = max(line2Points, 1) >= point(1) - tol;

    return e && f && g && h;
}
} // BoBRobotics
