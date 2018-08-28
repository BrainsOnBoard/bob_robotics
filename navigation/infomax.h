#pragma once

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <random>
#include <tuple>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/units.h"

// Local includes
#include "visual_navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace Eigen;
using namespace units::angle;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
class InfoMax
  : public VisualNavigationBase
{
public:
    InfoMax(const cv::Size unwrapRes,
            unsigned int scanStep = 1,
            float learningRate = 0.0001f,
            const filesystem::path &outputPath = "snapshots")
      : VisualNavigationBase(unwrapRes, scanStep, outputPath)
      , m_LearningRate(learningRate)
      , m_Weights(getInitialWeights(unwrapRes.width * unwrapRes.height, unwrapRes.width * unwrapRes.height))
    {}

    virtual void train(const cv::Mat &image, bool saveImage) override
    {
        assert(image.type() == CV_8U);

        const cv::Size &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);

        // Convert image to vector of floats
        const auto imageVector = getFloatVector(image);
        std::cout << "imageVector: " << imageVector << std::endl;

        const auto u = m_Weights * imageVector;

        const auto y = tanh(u.array());
        std::cout << "y: " << y << std::endl;

        // weights = weights + lrate/N * (eye(H)-(y+u)*u') * weights;
        const auto id = MatrixXf::Identity(m_Weights.rows(), m_Weights.rows());
        const auto sumYU = (y.array() + u.array()).matrix();
        const float learnRate = m_LearningRate / (float) imageVector.rows();
        m_Weights.array() += (learnRate * (id - sumYU * u.transpose()) * m_Weights).array();

        std::cout << "training!" << std::endl;
        if (saveImage) {
            saveSnapshot(m_SnapshotCount++, image);
        }
    }

    float decision(const cv::Mat &image) const
    {
        // Convert image to vector of floats
        const auto imageVector = getFloatVector(image);

        const auto decs = m_Weights * imageVector;
        return decs.array().abs().sum();
    }

    MatrixXf &getWeights()
    {
        return m_Weights;
    }

private:
    size_t m_SnapshotCount = 0;
    float m_LearningRate;
    MatrixXf m_Weights;

    static VectorXf getFloatVector(const cv::Mat &image)
    {
        Map<Matrix<uint8_t, Eigen::Dynamic, 1>> map(image.data, image.cols * image.rows);
        return map.cast<float>();
    }

    static MatrixXf getInitialWeights(int numInputs, int numHidden)
    {
        auto weights = MatrixXf(numInputs, numHidden);
        std::random_device rd;
        std::default_random_engine generator(0);
        std::normal_distribution<double> distribution;
        for (int y = 0; y < weights.rows(); y++) {
            for (int x = 0; x < weights.cols(); x++) {
                const double r = distribution(generator);
                weights(y, x) = r;
            }

            // Normalise mean and SD for row so mean == 0 and SD == 1
            auto row = weights.row(y);
            const auto mean = row.mean();
            const auto sd = sqrt(row.unaryExpr([mean] (double val) {
                const double diff = mean - val;
                return diff * diff;
            }).sum());
            row.array() -= mean;
            row.array() /= sd;
        }

        return weights;
    }
}; // InfoMax
} // Navigation
} // BoBRobotics