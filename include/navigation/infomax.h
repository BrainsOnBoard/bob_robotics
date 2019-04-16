#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/logging.h"
#include "insilico_rotater.h"
#include "visual_navigation_base.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <iostream>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
template<typename FloatType = float>
class InfoMax : public VisualNavigationBase
{
    using radian_t = units::angle::radian_t;
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorType = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

public:
    InfoMax(const cv::Size &unwrapRes,
            const MatrixType &initialWeights,
            FloatType learningRate = 0.0001)
      : VisualNavigationBase(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Weights(initialWeights)
    {}

    InfoMax(const cv::Size &unwrapRes, FloatType learningRate = 0.0001)
      : VisualNavigationBase(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Weights(getInitialWeights(unwrapRes.width * unwrapRes.height,
                                    1 + unwrapRes.width * unwrapRes.height))
    {}

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        calculateUY(image);
        trainUY();
    }

    virtual float test(const cv::Mat &image) const override
    {
        const auto decs = m_Weights * getFloatVector(image);
        return decs.array().abs().sum();
    }

    //! Generates new random weights
    virtual void clearMemory() override
    {
        m_Weights = getInitialWeights(m_Weights.cols(), m_Weights.rows());
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
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
        BOB_ASSERT(image.type() == CV_8UC1);

        const cv::Size &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);

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
        // Note that we transpose this matrix after normalisation
        MatrixType weights(numInputs, numHidden);

        LOG_INFO << "Seed for weights is: " << seed;

        std::default_random_engine generator(seed);
        std::normal_distribution<FloatType> distribution;
        for (int i = 0; i < numInputs; i++) {
            for (int j = 0; j < numHidden; j++) {
                weights(i, j) = distribution(generator);
            }
        }

        LOG_VERBOSE << "Initial weights" << std::endl << weights;

        // Normalise mean and SD for row so mean == 0 and SD == 1
        const auto means = weights.rowwise().mean();
        LOG_VERBOSE << "Means" << std::endl << means;

        weights.colwise() -= means;
        LOG_VERBOSE << "Weights after subtracting means" << std::endl << weights;

        LOG_VERBOSE << "New means" << std::endl << weights.rowwise().mean();

        const auto sd = matrixSD(weights);
        LOG_VERBOSE << "SD" << std::endl << sd;

        weights = weights.array().colwise() / sd;
        LOG_VERBOSE << "Weights after dividing by SD" << std::endl << weights;
        LOG_VERBOSE << "New SD" << std::endl << matrixSD(weights);

        return weights.transpose();
    }

private:
    size_t m_SnapshotCount = 0;
    FloatType m_LearningRate;
    MatrixType m_Weights;
    VectorType m_U, m_Y;

    static auto getFloatVector(const cv::Mat &image)
    {
        Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> map(image.data, image.cols * image.rows);
        return map.cast<FloatType>() / 255.0;
    }

    template<class T>
    static auto matrixSD(const T &mat)
    {
        return (mat.array() * mat.array()).rowwise().mean();
    }
}; // InfoMax

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMaxRotater
//------------------------------------------------------------------------
template<typename Rotater = InSilicoRotater, typename FloatType = float>
class InfoMaxRotater : public InfoMax<FloatType>
{
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;

public:
    InfoMaxRotater(const cv::Size &unwrapRes,
                   const MatrixType &initialWeights,
                   FloatType learningRate = 0.0001)
    :   InfoMax<FloatType>(unwrapRes, initialWeights, learningRate)
    {}

    InfoMaxRotater(const cv::Size &unwrapRes, FloatType learningRate = 0.0001)
    :   InfoMax<FloatType>(unwrapRes, learningRate)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    template<class... Ts>
    const std::vector<FloatType> &getImageDifferences(Ts &&... args) const
    {
        calcImageDifferences(std::forward<Ts>(args)...);
        return m_RotatedDifferences;
    }

    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        calcImageDifferences(std::forward<Ts>(args)...);

        const auto el = std::min_element(m_RotatedDifferences.cbegin(), m_RotatedDifferences.cend());
        int bestIndex = std::distance(m_RotatedDifferences.cbegin(), el);
        const cv::Size unwrapRes = this->getUnwrapResolution();
        if (bestIndex > (unwrapRes.width / 2)) {
            bestIndex -= unwrapRes.width;
        }

        using namespace units::angle;
        const radian_t heading = turn_t((double)bestIndex / (double)unwrapRes.width);

        return std::make_tuple(heading, *el, std::cref(m_RotatedDifferences));
    }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<class... Ts>
    void calcImageDifferences(Ts &&... args) const
    {
        // Object for rotating over images
        const cv::Size unwrapRes = this->getUnwrapResolution();
        auto rotater = Rotater::create(unwrapRes, this->getMaskImage(), std::forward<Ts>(args)...);

        // Ensure there's enough space in rotated differe
        m_RotatedDifferences.reserve(rotater.numRotations());
        m_RotatedDifferences.clear();

        // Populate rotated differences with results
        rotater.rotate([this] (const cv::Mat &image, auto, auto) {
            m_RotatedDifferences.push_back(this->test(image));
        });
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<FloatType> m_RotatedDifferences;
};
} // Navigation
} // BoBRobotics
