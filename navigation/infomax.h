#pragma once

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <random>
#include <tuple>
#include <utility>
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
                                    1 + unwrapRes.width * unwrapRes.height))
    {}

    virtual void train(const cv::Mat &image) override
    {
        calculateUY(image);
        trainUY();
    }

    FloatType decision(const cv::Mat &image) const
    {
        const auto decs = m_Weights * getFloatVector(image);
        return decs.array().abs().sum();
    }

    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        Rotater rotater(getUnwrapResolution(), getMaskImage(), std::forward<Ts>(args)...);
        std::vector<FloatType> outputs;
        outputs.reserve(rotater.max());
        rotater.rotate([this, &outputs] (const cv::Mat &image, auto, auto) {
            outputs.push_back(this->decision(image));
        });

        const auto el = std::min_element(outputs.begin(), outputs.end());
        size_t bestIndex = std::distance(outputs.begin(), el);
        if (bestIndex > outputs.size() / 2) {
            bestIndex -= outputs.size();
        }
        const radian_t heading = units::make_unit<turn_t>((double) bestIndex / (double) outputs.size());

        return std::make_tuple(heading, *el, std::move(outputs));
    }

    const MatrixType &getWeights() const
    {
        return m_Weights;
    }

#ifndef EXPOSE_INFOMAX_INTERNALS
    private:
#endif
    void trainUY()
    {
        // weights = weights + lrate/N * (eye(H)-(y+u)*u') * weights;
        const auto id = MatrixType::Identity(m_Weights.rows(), m_Weights.rows());
        const auto sumYU = (m_Y.array() + m_U.array()).matrix();
        const FloatType learnRate = m_LearningRate / (FloatType) m_U.rows();
        m_Weights.array() += (learnRate * (id - sumYU * m_U.transpose()) * m_Weights).array();
    }

    void calculateUY(const cv::Mat &image)
    {
        assert(image.type() == CV_8UC1);

        const cv::Size &unwrapRes = getUnwrapResolution();
        assert(image.cols == unwrapRes.width);
        assert(image.rows == unwrapRes.height);

        // Convert image to vector of floats
        m_U = m_Weights * getFloatVector(image);
        m_Y = tanh(m_U.array());
    }

    std::pair<VectorType, VectorType> getUY() const
    {
        // Copy the vectors
        return std::make_pair<>(m_U, m_Y);
    }

    static MatrixType getInitialWeights(const int numInputs,
                                        const int numHidden,
                                        const unsigned seed = std::random_device()())
    {
        MatrixType weights(numInputs, numHidden);

        std::cout << "Seed for weights is: " << seed << std::endl;

        std::default_random_engine generator(seed);
        std::normal_distribution<FloatType> distribution;
        for (int i = 0; i < numInputs; i++) {
            for (int j = 0; j < numHidden; j++) {
                weights(i, j) = distribution(generator);
            }
        }

        // std::cout << "Initial weights" << std::endl
        //           << weights << std::endl;

        // Normalise mean and SD for row so mean == 0 and SD == 1
        const auto means = weights.rowwise().mean();
        // std::cout << "Means" << std::endl
        //           << means << std::endl;

        weights.colwise() -= means;
        // std::cout << "Weights after subtracting means" << std::endl << weights << std::endl;

        // const auto newmeans = weights.rowwise().mean();
        // std::cout << "New means" << std::endl
        //           << newmeans << std::endl;

        const auto sd = matrixSD(weights);
        // std::cout << "SD" << std::endl
        //           << sd << std::endl;

        weights = weights.array().colwise() / sd;
        // std::cout << "Weights after dividing by SD" << std::endl
        //           << weights << std::endl;

        // const auto newsd = matrixSD(weights);
        // std::cout << "New SD" << std::endl
        //           << newsd << std::endl;

        return weights.transpose();
    }

private:
    size_t m_SnapshotCount = 0;
    FloatType m_LearningRate;
    MatrixType m_Weights;
    VectorType m_U, m_Y;

    static auto getFloatVector(const cv::Mat &image)
    {
        Map<Matrix<uint8_t, Dynamic, 1>> map(image.data, image.cols * image.rows);
        return map.cast<FloatType>() / 255.0;
    }

    template<class T>
    static auto matrixSD(const T &mat)
    {
        return (mat.array() * mat.array()).rowwise().mean();
    }
}; // InfoMax
} // Navigation
} // BoBRobotics
