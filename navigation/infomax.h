#pragma once

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <iostream>
#include <random>
#include <tuple>
#include <utility>

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
template<typename FloatType = float>
class InfoMax
  : public VisualNavigationBase
{
using MatrixType = Matrix<FloatType, Dynamic, Dynamic>;
using VectorType = Matrix<FloatType, Dynamic, 1>;

public:
    InfoMax<FloatType>(const cv::Size &unwrapRes,
                       const MatrixType &initialWeights,
                       unsigned int scanStep = 1,
                       FloatType learningRate = 0.0001,
                       const filesystem::path &outputPath = "snapshots")
      : VisualNavigationBase(unwrapRes, scanStep, outputPath)
      , m_LearningRate(learningRate)
      , m_Weights(initialWeights)
    {}

    InfoMax<FloatType>(const cv::Size &unwrapRes,
                       unsigned int scanStep = 1,
                       FloatType learningRate = 0.0001,
                       const filesystem::path &outputPath = "snapshots")
      : VisualNavigationBase(unwrapRes, scanStep, outputPath)
      , m_LearningRate(learningRate)
      , m_Weights(getInitialWeights(unwrapRes.width * unwrapRes.height,
                                    unwrapRes.width * unwrapRes.height))
    {}

    virtual void train(const cv::Mat &image, bool saveImage) override
    {
        VectorType u, y;
        std::tie(u, y) = getUY(image);
        train(u, y);

        if (saveImage) {
            saveSnapshot(m_SnapshotCount++, image);
        }
    }

    void train(const VectorType &u, const VectorType &y)
    {
        // weights = weights + lrate/N * (eye(H)-(y+u)*u') * weights;
        const auto id = MatrixType::Identity(m_Weights.rows(), m_Weights.rows());
        const auto sumYU = (y.array() + u.array()).matrix();
        const FloatType learnRate = m_LearningRate / (FloatType) u.rows();
        m_Weights.array() += (learnRate * (id - sumYU * u.transpose()) * m_Weights).array();
    }

    auto getUY(const cv::Mat &image)
    {
        assert(image.type() == CV_8UC1);

        const cv::Size &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);

        // Convert image to vector of floats
        const auto imageVector = getFloatVector(image);

        const auto u = m_Weights * imageVector;
        const auto y = tanh(u.array());

        return std::make_pair(u, y);
    }

    FloatType decision(const cv::Mat &image) const
    {
        // Convert image to vector of floats
        const auto imageVector = getFloatVector(image);

        const auto decs = m_Weights * imageVector;
        return decs.array().abs().sum();
    }

    MatrixType &getWeights()
    {
        return m_Weights;
    }

private:
    size_t m_SnapshotCount = 0;
    FloatType m_LearningRate;
    MatrixType m_Weights;

    static auto getFloatVector(const cv::Mat &image)
    {
        Map<Matrix<uint8_t, Dynamic, 1>> map(image.data, image.cols * image.rows);
        return map.cast<FloatType>() / 255.0;
    }

    static auto getInitialWeights(int numInputs, int numHidden)
    {
        MatrixType weights(numInputs, numHidden);

        std::random_device rd;
        std::default_random_engine generator(0);
        std::normal_distribution<FloatType> distribution;
        for (int y = 0; y < weights.rows(); y++) {
            for (int x = 0; x < weights.cols(); x++) {
                const FloatType r = distribution(generator);
                weights(y, x) = r;
            }

            // Normalise mean and SD for row so mean == 0 and SD == 1
            auto row = weights.row(y);
            const auto mean = row.mean();
            const auto sd = sqrt(row.unaryExpr([mean](FloatType val) {
                                        const auto diff = mean - val;
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