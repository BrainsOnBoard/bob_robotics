#pragma once

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <random>
#include <tuple>
#include <vector>

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/units.h"

// Local includes
#include "insilico_rotater.h"
#include "visual_navigation_base.h"

namespace BoBRobotics {
namespace Navigation {
using namespace Eigen;
using namespace units::angle;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
template<typename Rotater = InSilicoRotater, typename FloatType = float>
class InfoMax
  : public VisualNavigationBase
{
using MatrixType = Matrix<FloatType, Dynamic, Dynamic>;
using VectorType = Matrix<FloatType, Dynamic, 1>;

public:
    InfoMax<Rotater, FloatType>(const cv::Size &unwrapRes,
                                const MatrixType &initialWeights,
                                FloatType learningRate = 0.0001)
      : VisualNavigationBase(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Weights(initialWeights)
    {}

    InfoMax<Rotater, FloatType>(const cv::Size &unwrapRes,
                                FloatType learningRate = 0.0001)
      : VisualNavigationBase(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Weights(getInitialWeights(unwrapRes.width * unwrapRes.height,
                                    unwrapRes.width * unwrapRes.height))
    {}

    virtual void train(const cv::Mat &image) override
    {
        VectorType u, y;
        std::tie(u, y) = getUY(image);
        trainUY(u, y);
    }

    FloatType decision(const cv::Mat &image) const
    {
        // Convert image to vector of floats
        const auto imageVector = getFloatVector(image);

        const auto decs = m_Weights * imageVector;
        return decs.array().abs().sum();
    }

    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        Rotater rotater(getUnwrapResolution(), getMaskImage(), std::forward<Ts>(args)...);
        std::vector<FloatType> outputs;
        outputs.reserve(rotater.max());
        rotater.rotate([this, &outputs] (const cv::Mat &image, auto, auto) {
            outputs.push_back(decision(image));
        });

        const auto el = std::min_element(outputs.begin(), outputs.end());
        size_t bestIndex = std::distance(outputs.begin(), el);
        if (bestIndex > outputs.size() / 2) {
            bestIndex -= outputs.size();
        }
        const radian_t heading = units::make_unit<turn_t>((double) bestIndex / (double) outputs.size());

        return std::make_tuple(heading, *el, std::move(outputs));
    }

    MatrixType &getWeights()
    {
        return m_Weights;
    }

#ifndef EXPOSE_INFOMAX_INTERNALS
    private:
#endif
    void trainUY(const VectorType &u, const VectorType &y)
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

        return std::make_pair(std::move(u), std::move(y));
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

        std::default_random_engine generator;
        std::normal_distribution<FloatType> distribution;
        for (int y = 0; y < weights.rows(); y++) {
            for (int x = 0; x < weights.cols(); x++) {
                const FloatType r = distribution(generator);
                weights(y, x) = r;
            }

            // Normalise mean and SD for row so mean == 0 and SD == 1
            auto row = weights.row(y);
            const auto mean = row.mean();
            const auto diffSq = [mean](FloatType val) {
                const auto diff = mean - val;
                return diff * diff;
            };
            const auto sd = sqrt(row.unaryExpr(diffSq).sum());
            row.array() -= mean;
            row.array() /= sd;
        }

        return weights;
    }
}; // InfoMax
} // Navigation
} // BoBRobotics