#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <algorithm>
#include <bitset>


namespace BoBRobotics {
namespace ImgProc {
namespace DCTHash {
//! computes the DCT hash
inline std::bitset<64>
computeHash(const cv::Mat &in)
{
    cv::Mat dct_mat;
    cv::dct(in, dct_mat);
    cv::Mat rect(dct_mat, { 0, 0, 8, 8 });             // we only need the low 8x8 frequencies

    std::array<float, 33> sorted;
    std::partial_sort_copy(rect.begin<float>(), rect.end<float>(), sorted.begin(), sorted.end());
    const float median = (sorted[31] + sorted[32]) /2;


    std::bitset<64> binary;
    for (size_t i = 0; i < 64; i++) {
        if (reinterpret_cast<float *>(rect.data)[i] > median) {
            binary.set(i, 1);
        }
    }
    return binary;
}

//! computes the hamming distance between 2 image hashes
inline int
distance(const std::bitset<64> &hash1, const std::bitset<64> &hash2)
{
    const std::bitset<64> delta = hash1 ^ hash2;
    const int distance = delta.count();
    return distance;
}

} // DCTHash
} // ImgProc
} // BoBRobotics
