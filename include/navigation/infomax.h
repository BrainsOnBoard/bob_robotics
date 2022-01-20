#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "imgproc/mask.h"
#include "navigation/insilico_rotater.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <exception>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

#define INFOMAX_DEFAULT_LEARNING_RATE 0.01
#define DEFAULT_TANH_SCALING_FACTOR 0.1

namespace BoBRobotics {
namespace Navigation {
class WeightsBlewUpError
  : std::exception
{};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
template<typename FloatType = float>
class InfoMax
{
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorType = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

public:
    InfoMax(const cv::Size &unwrapRes,
            MatrixType initialWeights,
            FloatType learningRate = INFOMAX_DEFAULT_LEARNING_RATE,
            FloatType tanhScalingFactor = DEFAULT_TANH_SCALING_FACTOR)
      : m_UnwrapRes(unwrapRes)
      , m_LearningRate(learningRate)
      , m_TanhScalingFactor(tanhScalingFactor)
      , m_Weights(std::move(initialWeights))
    {
        BOB_ASSERT(m_Weights.cols() == unwrapRes.width * unwrapRes.height);
    }

    InfoMax(const cv::Size &unwrapRes,
            FloatType learningRate = INFOMAX_DEFAULT_LEARNING_RATE,
            FloatType tanhScalingFactor = DEFAULT_TANH_SCALING_FACTOR)
      : InfoMax(unwrapRes,
                generateInitialWeights(unwrapRes.width * unwrapRes.height,
                                       unwrapRes.width * unwrapRes.height),
                learningRate,
                tanhScalingFactor)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void train(const cv::Mat &image, const ImgProc::Mask& = ImgProc::Mask{})
    {
        calculateUY(image);
        trainUY();
    }

    float test(const cv::Mat &image, const ImgProc::Mask& = ImgProc::Mask{}) const
    {
        const auto decs = m_Weights * getNetInputs(image);
        return decs.array().abs().sum();
    }

    //! Generates new random weights
    void clearMemory()
    {
        m_Weights = generateInitialWeights(m_Weights.cols(), m_Weights.rows());
    }


    const MatrixType &getWeights() const
    {
        return m_Weights;
    }

    static MatrixType generateInitialWeights(const int numInputs,
                                             const int numHidden,
                                             const unsigned seed = std::random_device()())
    {
        // Note that we transpose this matrix after normalisation
        MatrixType weights(numInputs, numHidden);

        LOG_INFO << "Seed for weights is: " << seed;

        std::default_random_engine generator(seed);
        std::normal_distribution<FloatType> distribution;
        std::generate_n(weights.data(), weights.size(), [&]() { return distribution(generator); });

        // Normalise mean and SD for row so mean == 0 and SD == 1
        const auto means = weights.rowwise().mean();
        weights.colwise() -= means;

        const auto sd = matrixSD(weights);
        weights = weights.array().colwise() / sd;

        return weights.transpose();
    }

    //! Get the resolution of images
    const cv::Size &getUnwrapResolution() const { return m_UnwrapRes; }

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

        /*
         * If the learning rate is too high, we may end up with NaNs in our
         * weight matrix, which will silently muck up subsequent calculations.
         * I don't *think* this will ever be an issue with a sensibly small
         * learning rate, but if so, the learning rate could be reduced at this
         * point instead of just bailing out.
         *
         * So if there are any NaNs then throw an error.
         */
        if (!(m_Weights.array() == m_Weights.array()).all()) {
            throw WeightsBlewUpError{};
        }
    }

    void calculateUY(const cv::Mat &image)
    {
        BOB_ASSERT(image.type() == CV_8UC1);

        const cv::Size &unwrapRes = getUnwrapResolution();
        BOB_ASSERT(image.cols == unwrapRes.width);
        BOB_ASSERT(image.rows == unwrapRes.height);

        // Convert image to vector of floats
        m_U = m_Weights * getNetInputs(image) * m_TanhScalingFactor;
        m_Y = tanh(m_U.array());
    }

    std::pair<VectorType, VectorType> getUY() const
    {
        // Copy the vectors
        return std::make_pair<>(m_U, m_Y);
    }

private:
    const cv::Size m_UnwrapRes;
    const FloatType m_LearningRate, m_TanhScalingFactor;
    MatrixType m_Weights;
    VectorType m_U, m_Y;

    //! Normalises the input image to a vector of z-scores
    static VectorType getNetInputs(const cv::Mat &image)
    {
        Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> map(image.data, image.cols * image.rows);
        const auto v = map.cast<FloatType>();
        const FloatType mean = v.mean();

        const auto vNorm = v.array() - mean;
        const FloatType sd = std::sqrt((vNorm * vNorm).mean());
        return vNorm / sd;
    }

    //! NB: This only works if mat's column means are zero!
    template<class T>
    static auto matrixSD(const T &mat)
    {
        return (mat.array() * mat.array()).rowwise().mean().sqrt();
    }
}; // InfoMax

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMaxRotater
//------------------------------------------------------------------------
template<typename FloatType = float>
class InfoMaxRotater : public InfoMax<FloatType>
{
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;

public:
    InfoMaxRotater(const cv::Size &unwrapRes,
                   MatrixType initialWeights,
                   FloatType learningRate = INFOMAX_DEFAULT_LEARNING_RATE,
                   FloatType tanhScalingFactor = DEFAULT_TANH_SCALING_FACTOR)
    :   InfoMax<FloatType>(unwrapRes, std::move(initialWeights), learningRate, tanhScalingFactor)
    {}

    InfoMaxRotater(const cv::Size &unwrapRes,
                   FloatType learningRate = INFOMAX_DEFAULT_LEARNING_RATE,
                   FloatType tanhScalingFactor = DEFAULT_TANH_SCALING_FACTOR)
    :   InfoMax<FloatType>(unwrapRes, learningRate, tanhScalingFactor)
    {}

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    template<class... Ts>
    const std::vector<FloatType> &getImageDifferences(const cv::Mat &image, ImgProc::Mask mask, Ts &&... args) const
    {
        auto rotater = InSilicoRotater::create(this->getUnwrapResolution(), mask, image, std::forward<Ts>(args)...);
        calcImageDifferences(rotater);
        return m_RotatedDifferences;
    }

    template<class... Ts>
    const std::vector<FloatType> &getImageDifferences(const cv::Mat &image, Ts &&... args) const
    {
        return getImageDifferences(image, ImgProc::Mask{}, std::forward<Ts>(args)...);
    }

    template<class... Ts>
    auto getHeading(const cv::Mat &image, ImgProc::Mask mask, Ts &&... args) const
    {
        using radian_t = units::angle::radian_t;

        const cv::Size unwrapRes = this->getUnwrapResolution();
        auto rotater = InSilicoRotater::create(unwrapRes, mask, image, std::forward<Ts>(args)...);
        calcImageDifferences(rotater);

        // Find index of lowest difference
        const auto el = std::min_element(m_RotatedDifferences.cbegin(), m_RotatedDifferences.cend());
        const size_t bestIndex = std::distance(m_RotatedDifferences.cbegin(), el);

        // Convert this to an angle
        radian_t heading = rotater.columnToHeading(bestIndex);
        while (heading <= -180_deg) {
            heading += 360_deg;
        }
        while (heading > 180_deg) {
            heading -= 360_deg;
        }

        return std::make_tuple(heading, *el, std::cref(m_RotatedDifferences));
    }

    template<class... Ts>
    auto getHeading(const cv::Mat &image, Ts &&... args) const
    {
        return getHeading(image, ImgProc::Mask{}, std::forward<Ts>(args)...);
    }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    template<typename R>
    void calcImageDifferences(R &rotater) const
    {
        // Ensure there's enough space in m_RotatedDifferences
        m_RotatedDifferences.resize(rotater.numRotations());

        // Populate rotated differences with results
        rotater.rotate([this] (const cv::Mat &image, const ImgProc::Mask &, size_t i) {
            m_RotatedDifferences[i] = this->test(image);
        });
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<FloatType> m_RotatedDifferences;
};
} // Navigation
} // BoBRobotics
