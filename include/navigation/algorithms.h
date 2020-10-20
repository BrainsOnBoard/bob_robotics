#pragma once

// BoB robotics includes
#include "imgproc/roll_image.h"
#include "navigation/differencers_new.h"

// Third-party includes
#include "third_party/units.h"

// Eigen
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <utility>

namespace BoBRobotics {
namespace Navigation {

// template<class Iter>
// auto getBestRotation(Iter begin, Iter end, size_t width=std::distance(begin, end))
// {
//     using namespace units::angle;

//     const auto min = std::min_element(begin, end);
//     const degree_t angle = turn_t{ (double) std::distance(begin, min) /
//                                    (double) width };
//     return std::make_pair(angle, *min);
// }

// template<class Container>
// auto getBestRotation(const Container &vals, size_t width=std::distance(begin, end))
// {
//     return getBestRotation(std::cbegin(vals), std::cend(vals), width);
// }

template<class T>
class RowIterator {
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = T;
    using pointer = T*;
    using reference = T&;

    RowIterator(T *data, const size_t stride=1)
      : m_Data(data)
      , m_Stride(stride)
    {}

    T *operator*() const { return m_Data; }

    auto &operator++()
    {
        m_Data += m_Stride;
        return *this;
    }

    bool operator==(const T *other) { return m_Data == other; }
    bool operator==(RowIterator<T> other) { return m_Data == other.m_Data; }

    template<class U>
    bool operator!=(const U val) { return !(*this == val); }

private:
    T *m_Data;
    const size_t m_Stride;
};

template <
    typename T,
    typename std::enable_if_t<std::is_base_of<Eigen::MatrixBase<T>, T>::value, int> = 0
>
auto __getIterator(T &values)
{
    return RowIterator<float>{ values.data(), (size_t)values.cols() };
}

template <
    typename T,
    typename std::enable_if_t<!std::is_base_of<Eigen::MatrixBase<T>, T>::value, int> = 0
>
auto __getIterator(T &values)
{
    return std::begin(values);
}

template<class It1, class It2, class Func>
void
calculateRIDFMulti(const cv::Mat &image, const It1 snapBegin, const It1 snapEnd,
                   It2 diffRowsBegin, Func calcSnapshotDifference,
                   size_t columnStep, size_t columnBegin, size_t columnEnd)
{
    auto snapshotComp = [&](const cv::Mat &rotImage, size_t rotation) {
        auto diffsIter = diffRowsBegin;
        for (auto snap = snapBegin; snap != snapEnd; ++snap, ++diffsIter) {
            (*diffsIter)[rotation] = calcSnapshotDifference(rotImage, *snap);
        }
    };
    ImgProc::forEachRotation<float>(image, snapshotComp, columnStep, columnBegin, columnEnd);
}

template<class ImageType, class Container, class Differences, class Func>
void
calculateRIDFMulti(const ImageType &image, const Container &snapshots,
                   Differences &differences, Func calcSnapshotDifference)
{
    auto iterPair = __getIterator(differences);
    calculateRIDFMulti(image, std::cbegin(snapshots), std::cend(snapshots),
                       iterPair.first, iterPair.second, calcSnapshotDifference);
}

template<class Func>
const auto &
calculateRIDF(const cv::Mat &image, const cv::Mat &snapshot,
              Func calcSnapshotDifference)
{
    static std::vector<float> differences;
    differences.resize(image.cols);
    calculateRIDFMulti(image, &snapshot, &snapshot + 1, &differences,
                       calcSnapshotDifference, 1, 0, image.cols);
    return differences;
}

inline const auto &
calculateRIDF(const cv::Mat &image, const cv::Mat &snapshot)
{
    return calculateRIDF(image, snapshot, AbsDiff{});
}

} // Navigation
} // BoBRobotics
