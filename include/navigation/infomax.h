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

namespace BoBRobotics {
namespace Navigation {
class WeightsBlewUpError
  : std::runtime_error
{
public:
    WeightsBlewUpError()
      : std::runtime_error("InfoMax net's weights blew up")
    {}
};

enum class Normalisation
{
    None,
    ZScore
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
template<typename FloatType = float>
class InfoMax
{
public:
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorType = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

    static constexpr FloatType DefaultLearningRate{ 0.97 };

    InfoMax(const cv::Size &unwrapRes,
            FloatType learningRate,
            Normalisation normalisation,
            MatrixType initialWeights)
      : m_UnwrapRes(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Normalisation(normalisation)
      , m_Weights(std::move(initialWeights))
    {
        BOB_ASSERT(m_Weights.cols() == unwrapRes.width * unwrapRes.height);
    }

    InfoMax(const cv::Size &unwrapRes, unsigned int numHidden,
            FloatType learningRate = DefaultLearningRate,
            Normalisation normalisation = Normalisation::None)
      : InfoMax(unwrapRes, learningRate, normalisation,
                generateInitialWeights(unwrapRes.width * unwrapRes.height, 
                                       numHidden))
    {}

    InfoMax(const cv::Size &unwrapRes,
            FloatType learningRate = DefaultLearningRate,
            Normalisation normalisation = Normalisation::None)
      : InfoMax(unwrapRes, unwrapRes.width * unwrapRes.height, learningRate, 
                normalisation)
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
        return getNetOutputs(image).array().abs().sum();
    }

    //! Generates new random weights
    void clearMemory()
    {
        m_Weights = generateInitialWeights(getNumInputs(), getNumHidden());
    }

    float getLearningRate() const
    {
        return m_LearningRate;
    }

    Normalisation getNormalisationMethod() const
    {
        return m_Normalisation;
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
        // weights = weights + lrate/N.M * (eye(M)-(y+u)*u') * weights;
        // **NOTE** additional scaling after M from Azevedo et al. 2023
        const auto id = MatrixType::Identity(getNumHidden(), getNumHidden());
        const FloatType learnRate = m_LearningRate / (FloatType)(getNumInputs() * getNumHidden());

        // **NOTE** middle term evaluates to MxM matrix, m_Weights are M*N,
        // meaning result is M*N as required for adding back to m_Weights
        m_Weights += (learnRate * (id - ((m_Y + m_U) * m_U.transpose())) * m_Weights);

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
        // **NOTE** m_Weights is MxN matrix and input is a Nx1 column vector so m_U and m_Y and Mx1 column vectors
        m_U = m_Weights * getNetInputs(image);
        m_Y = m_U.array().tanh();
    }

    std::pair<VectorType, VectorType> getUY() const
    {
        // Copy the vectors
        return std::make_pair<>(m_U, m_Y);
    }

    VectorType getNetOutputs(const cv::Mat &image) const
    {
        return m_Weights * getNetInputs(image);
    }

private:
    const cv::Size m_UnwrapRes;
    const FloatType m_LearningRate;
    const Normalisation m_Normalisation;
    MatrixType m_Weights;
    VectorType m_U, m_Y;

    Eigen::Index getNumInputs() const
    {
        return m_Weights.cols();
    }

    Eigen::Index getNumHidden() const
    {
        return m_Weights.rows();
    }

    //! Converts image to VectorType and normalises
    VectorType getNetInputs(const cv::Mat &image) const
    {
        Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> map(image.data, image.cols * image.rows);
        const auto vec = map.cast<FloatType>();

        if(m_Normalisation == Normalisation::None) {
            return vec / 255.0;
        }
        else {
            const FloatType mean = vec.mean();

            const auto vNorm = vec.array() - mean;
            const FloatType sd = std::sqrt((vNorm * vNorm).mean());
            return vNorm / sd;
        }
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
public:
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;
    
    using InfoMax<FloatType>::InfoMax;

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

template<class T>
constexpr T InfoMax<T>::DefaultLearningRate;

} // Navigation
} // BoBRobotics
