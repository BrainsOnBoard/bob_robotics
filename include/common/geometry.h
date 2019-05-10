#pragma once

// BoB robotics includes
#include "assert.h"
#include "circstat.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <ostream>
#include <limits>
#include <vector>

namespace BoBRobotics {
template<class T>
using EigenSTDVector = std::vector<T, Eigen::aligned_allocator<T>>;

inline double
distance2D(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    return hypot(p2.x() - p1.x(), p2.y() - p1.y());
}

template<class T = double>
constexpr inline T inf()
{
    return std::numeric_limits<T>::infinity();
}

constexpr inline bool
approxEq(double a, double b)
{
    return std::abs(a - b) < 1e-5;
}

struct StraightLine
{
    double m, c;

    constexpr StraightLine()
      : m(0)
      , c(0)
    {}

    constexpr StraightLine(double m, double c)
      : m(m)
      , c(c)
    {}

    auto perpendicular(const Eigen::Vector2d &point) const
    {
        double mPerp, cPerp;
        BOB_ASSERT(isPointOnLine(point));

        if (isVertical()) {
            mPerp = 0.0;
            cPerp = point.y();
        } else if (isHorizontal()) {
            mPerp = inf();
            cPerp = point.x();
        } else {
            mPerp = -1.0 / m;
            cPerp = point.y() - mPerp * point.x();
        }
        return StraightLine(mPerp, cPerp);
    }

    bool isHorizontal() const { return approxEq(m, 0.0); }
    bool isVertical() const { return std::isinf(m); }

    bool isParallel(const StraightLine &line) const
    {
        return (isVertical() && line.isVertical()) || approxEq(m, line.m);
    }

    bool isPointOnLine(const Eigen::Vector2d &point) const
    {
        if (isVertical()) {
            return approxEq(c, point.x());
        } else {
            return approxEq(point.y(), getY(point.x()));
        }
    }

    Eigen::Vector2d intersection(const StraightLine &line) const
    {
        double x, y;
        BOB_ASSERT(!isParallel(line));

        if (isVertical()) {
            x = c;
            y = line.getY(x);
        } else if (isHorizontal()) {
            y = c;
            x = line.getX(y);
        } else {
            x = (line.c - c) / (m - line.m);
            y = getY(x);
        }

        return { x, y };
    }

    double angle() const
    {
        return atan(m);
    }

    Eigen::Vector2d pointAlong(const Eigen::Vector2d &startPoint, double distance) const
    {
        BOB_ASSERT(isPointOnLine(startPoint));

        const auto th = angle();
        const auto dy = distance * sin(th);
        const auto dx = distance * cos(th);
        return { startPoint.x() + dx, startPoint.y() + dy };
    }

    double getX(double y) const
    {
        if (isVertical()) {
            return c;
        } else {
            return (y - c) / m;
        }
    }

    double getY(double x) const
    {
        return m * x + c;
    }

    bool operator==(const StraightLine &line) const
    {
        if (!approxEq(c, line.c)) {
            return false;
        } else if (isVertical()) {
            return line.isVertical();
        } else {
            return (approxEq(m, line.m) && approxEq(c, line.c));
        }
    }

    static auto fromPoints(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
        double m, c;

        const auto dy = p2.y() - p1.y();
        const auto dx = p2.x() - p1.x();
        if (approxEq(dx, 0.0)) {
            // Vertical line
            m = inf();
            c = p1.x();
        } else {
            m = dy / dx;

            if (approxEq(m, 0.0)) {
                // Horizontal line
                c = p1.y();
            } else {
                c = p2.y() - m * p2.x();
            }
        }

        return StraightLine(m, c);
    }
};

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

const auto min = [](const auto &line, int index) {
    return line.col(index).minCoeff();
};
const auto max = [](const auto &line, int index) {
    return line.col(index).maxCoeff();
};

inline bool
calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1, const StraightLine &lineEq2)
{
    const auto lineEq1 = StraightLine::fromPoints(line1.row(0), line1.row(1));
    point = lineEq1.intersection(lineEq2);

    // Margin of error for floating-point numbers
    constexpr double tol = std::numeric_limits<double>::epsilon();

    // Check that the point is on line 1
    const bool a = min(line1, 0) <= point(0) + tol;
    const bool b = max(line1, 0) >= point(0) - tol;
    const bool c = min(line1, 1) <= point(1) + tol;
    const bool d = max(line1, 1) >= point(1) - tol;
    return a && b && c && d;
}

inline bool
calculateIntersection(Eigen::Vector2d &point, const Eigen::Matrix2d &line1, const Eigen::Matrix2d &line2)
{
    const auto lineEq2 = StraightLine::fromPoints(line2.row(0), line2.row(1));
    if (!calculateIntersection(point, line1, lineEq2)) {
        return false;
    }

    // Margin of error for floating-point numbers
    constexpr double tol = std::numeric_limits<double>::epsilon();

    // Check that the point is on line 2
    const bool e = min(line2, 0) <= point(0) + tol;
    const bool f = max(line2, 0) >= point(0) - tol;
    const bool g = min(line2, 1) <= point(1) + tol;
    const bool h = max(line2, 1) >= point(1) - tol;

    return e && f && g && h;
}
} // BoBRobotics

inline std::ostream &
operator<<(std::ostream &os, const BoBRobotics::StraightLine &line)
{
    if (line.isVertical()) {
        os << "x = " << line.c << " (m=" << line.m << ")";
    } else if (line.isHorizontal()) {
        os << "y = " << line.c;
    } else {
        os << "y = " << line.m << "* x + " << line.c;
    }
    return os;
}
