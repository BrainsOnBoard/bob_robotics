#include "common.h"

// BoB robotics includes
#include "common/geometry.h"

#include <array>
#include <algorithm>

using SL = StraightLine;

template<int Rows1, int Cols1, int Rows2, int Cols2>
void EXPECT_EIGEN_EQ(const Eigen::Matrix<double, Rows1, Cols1> &m1, const Eigen::Matrix<double, Rows2, Cols2> &m2)
{
    ASSERT_EQ(m1.size(), m2.size());
    for (int i = 0; i < m1.rows(); i++) {
        for (int j = 0; j < m1.cols(); j++) {
            EXPECT_DOUBLE_EQ(m1(i, j), m2(i, j));
        }
    }
}

TEST(Geometry, ResizeObjectBy) {
    Eigen::Matrix<double, 4, 2> object;
    object << -1, -0.5,
              -1,  0.5,
               1,  0.5,
               1, -0.5;
    Eigen::Matrix<double, 4, 2> objectResized;
    objectResized << -1.5, -1,
                     -1.5,  1,
                      1.5,  1,
                      1.5, -1;
    resizePolygonBy(object, 0.5_m);

    EXPECT_EIGEN_EQ(object, objectResized);
}

TEST(Geometry, StraightLineFromPoints) {
    constexpr int numTests = 4;
    Eigen::Matrix<double, numTests, 2> startPoints, endPoints;
    startPoints << 1, -0.5,
                  -1, -0.5,
                  -1,  0.5,
                   1,  0.5,
    endPoints << -1, -0.5,
                 -1,  0.5,
                  1,  0.5,
                  1, -0.5;

    constexpr std::array<SL, numTests> lines = {
        SL{ 0.0, -0.5 },
        SL{ inf(), -1.0 },
        SL{ 0.0, 0.5 },
        SL{ inf(), 1.0 }
    };

    EXPECT_EQ(inf(), inf());
    for (int i = 0; i < numTests; i++) {
        const auto testLine = SL::fromPoints(startPoints.row(i), endPoints.row(i));
        const auto &actualLine = lines[static_cast<size_t>(i)];
        EXPECT_EQ(actualLine, testLine);
    }
}

TEST(Geometry, Perpendicular) {
    constexpr int numTests = 4;

    Eigen::Matrix<double, numTests, 2> vertices;
    vertices << -1, -0.5,
                -1,  0.5,
                 1,  0.5,
                 1, -0.5;
    constexpr std::array<SL, numTests> lines = {
        SL{ 0.0, -0.5 },
        SL{ inf(), -1.0 },
        SL{ 0.0, 0.5 },
        SL{ inf(), 1.0 }
    };
    const std::array<SL, numTests> actualLines = {
        SL{ inf(), vertices(0, 0) },
        SL{ 0.0, vertices(1, 1) },
        SL{ inf(), vertices(2, 0) },
        SL{ 0.0, vertices(3, 1) }
    };

    for (size_t i = 0; i < numTests; i++) {
        const auto testLine = lines[i].perpendicular(vertices.row(static_cast<int>(i)));
        EXPECT_EQ(actualLines[i], testLine);
    }
}

TEST(Geometry, PointOnPerpendicular) {
    constexpr int numTests = 4;

    Eigen::Matrix<double, numTests, 2> vertices;
    vertices << -1, -0.5,
                -1,  0.5,
                 1,  0.5,
                 1, -0.5;
    constexpr std::array<SL, numTests> lines = {
        SL{ 0.0, -0.5 },
        SL{ inf(), -1.0 },
        SL{ 0.0, 0.5 },
        SL{ inf(), 1.0 }
    };

    for (size_t i = 0; i < numTests; i++) {
        const auto startPoint = vertices.row(static_cast<int>(i));
        const auto testLine = lines[i].perpendicular(startPoint);
        EXPECT_TRUE(testLine.isPointOnLine(startPoint));
    }
}
