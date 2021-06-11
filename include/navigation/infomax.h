#pragma once

// BoB robotics includes
#include "common/circstat.h"
#include "common/macros.h"
#include "imgproc/mask.h"
#include "navigation/insilico_rotater.h"
#include "navigation/visual_navigation_base.h"

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
  : std::exception
{};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::InfoMax
//------------------------------------------------------------------------
template<typename FloatType = float>
class InfoMax : public VisualNavigationBase
{
    using MatrixType = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;
    using VectorType = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

public:
    InfoMax(const cv::Size &unwrapRes,
            const MatrixType &initialWeights,
            FloatType learningRate = 0.0001)
      : VisualNavigationBase(unwrapRes)
      , m_LearningRate(learningRate)
      , m_Weights(initialWeights)
    {
        BOB_ASSERT(initialWeights.cols() == unwrapRes.width * unwrapRes.height);
    }

    InfoMax(const cv::Size &unwrapRes, FloatType learningRate = 0.0001)
      : InfoMax(unwrapRes,
                generateInitialWeights(unwrapRes.width * unwrapRes.height,
                                       unwrapRes.width * unwrapRes.height),
                learningRate)
    {}

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    virtual void train(const cv::Mat &image) override
    {
        getMask().apply(image, m_ScratchImage);
        calculateUY(image);
        trainUY();
    }

    virtual float test(const cv::Mat &image) const override
    {
        return testSum(image, getMask()) / static_cast<float>(0xff * image.rows * image.cols);
    }

    float test(const cv::Mat &image, const ImgProc::Mask &extraMask) const
    {
        static cv::Mat scratchImage;
        static ImgProc::Mask scratchMask;
#pragma omp threadprivate(scratchImage, scratchMask)

        extraMask.combine(getMask(), scratchMask);
        scratchMask.apply(image, scratchImage);

        const auto n = extraMask.countUnmaskedPixels(image.size());
        return testSum(scratchImage, scratchMask) / static_cast<float>(0xff * n);
        }

    //! Generates new random weights
    virtual void clearMemory() override
    {
        m_Weights = generateInitialWeights(m_Weights.cols(), m_Weights.rows());
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    const auto &getDecision(const cv::Mat &image, const ImgProc::Mask &mask) const
    {
        static thread_local VectorType scratchVector;

        scratchVector = m_Weights * getFloatVector(image);
        applyMask(scratchVector, mask);
        return scratchVector;
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
        m_U = m_Weights * getFloatVector(image);
        m_Y = tanh(m_U.array());
    }

    std::pair<VectorType, VectorType> getUY() const
    {
        // Copy the vectors
        return { m_U, m_Y };
    }

private:
    size_t m_SnapshotCount = 0;
    FloatType m_LearningRate;
    MatrixType m_Weights;
    VectorType m_U, m_Y;
    cv::Mat m_ScratchImage;

    float testSum(const cv::Mat &image, const ImgProc::Mask &mask) const
    {
        return getDecision(image, mask).array().abs().sum();
    }

    static void applyMask(VectorType &vec, const ImgProc::Mask &mask)
    {
        if (mask.empty()) {
            return;
        }

        const auto &maskMat = mask.get();
        const auto maskBegin = maskMat.datastart;
        for (int i = 0; i < maskMat.rows * maskMat.cols; i++) {
            if (!maskBegin[i]) {
                vec(i) = 0;
            }
        }
    }

    static auto getFloatVector(const cv::Mat &image)
    {
        BOB_ASSERT(image.type() == CV_8UC1);
        Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> map(image.data, image.cols * image.rows);
        return map.cast<FloatType>() / FloatType{ 0xff };
    }

    template<class T>
    static auto matrixSD(const T &mat)
    {
        return (mat.array() * mat.array()).rowwise().mean().sqrt();
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
        auto rotater = Rotater::create(this->getUnwrapResolution(), this->getMask(), std::forward<Ts>(args)...);
        calcImageDifferences(rotater);
        return m_RotatedDifferences;
    }

    template<class... Ts>
    auto getHeading(Ts &&... args) const
    {
        using radian_t = units::angle::radian_t;

        const cv::Size unwrapRes = this->getUnwrapResolution();
        auto rotater = Rotater::create(unwrapRes, this->getMask(), std::forward<Ts>(args)...);
        calcImageDifferences(rotater);

        // Find index of lowest difference
        const auto el = std::min_element(m_RotatedDifferences.cbegin(), m_RotatedDifferences.cend());
        const size_t bestIndex = std::distance(m_RotatedDifferences.cbegin(), el);

        // Convert this to an angle
        radian_t heading = rotater.columnToHeading(bestIndex);
        heading = normaliseAngle180(heading);
        return std::make_tuple(heading, *el, std::cref(m_RotatedDifferences));
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
        rotater.rotate([this] (const cv::Mat &image, const ImgProc::Mask &mask, size_t i) {
            m_RotatedDifferences[i] = this->test(image, mask);
        });
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable std::vector<FloatType> m_RotatedDifferences;
};
} // Navigation
} // BoBRobotics
