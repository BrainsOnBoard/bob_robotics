#pragma once

// BoB robotics includes
#include "common/geometry.h"

#include <algorithm>

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

// Currently broken
//TEST(Geometry, ResizeObjectBy) {
//    Eigen::Matrix<double, 4, 2> object;
//    object << -1, -0.5,
//              -1,  0.5,
//               1,  0.5,
//               1, -0.5;
//    Eigen::Matrix<double, 4, 2> objectResized;
//    objectResized << -1.5, -1,
//                     -1.5,  1,
//                      1.5,  1,
//                      1.5, -1;
//    resizePolygonBy(object, 0.5_m);

//    EXPECT_EIGEN_EQ(object, objectResized);
//}
