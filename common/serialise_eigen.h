#pragma once

// BoB robotics
#include "assert.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Core>

namespace cv {
inline void write(cv::FileStorage &fs, const std::string&, const Eigen::MatrixXd &matrix)
{
    fs << cv::Mat((int) matrix.rows(), (int) matrix.cols(), CV_64F, (void *) matrix.data());
}

inline void read(const cv::FileNode &node, Eigen::MatrixXd &matrix, Eigen::MatrixXd defaultValue = Eigen::MatrixXd())
{
    if(node.empty()) {
        matrix = defaultValue;
    }
    else {
        cv::Mat matrixCV;
        node >> matrixCV;
        BOB_ASSERT(matrixCV.type() == CV_64F);
        Eigen::Map<Eigen::MatrixXd> map((double *) matrixCV.data, matrixCV.rows, matrixCV.cols);
        matrix = map;
    }
}
}
