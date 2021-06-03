#pragma once

// Standard C++ includes
#include <iostream>
#include <string>
#include <algorithm>
#include <bitset>
// Standard C includes
#include <cmath>

//opencv
#include <opencv2/opencv.hpp>

namespace DCTHash
{
    //! computes the DCT hash
    inline static std::bitset<64> computeHash(const cv::Mat &in) {

        cv::Mat dct_mat;
        cv::dct(in, dct_mat);
        cv::Mat rect(dct_mat,{ 0, 0, 8, 8});                // we only need the low 8x8 frequencies
        std::sort(rect.begin<float>(),rect.end<float>());   // sorting to get median value
        const float median= rect.at<float>(-1+rect.size().height/2,rect.size().width-1); 
        rect = {dct_mat,{ 0, 0, 8, 8}};                     // re-assigning so we get the correct order back
	
        std::bitset<64> binary;
        for (size_t i = 0; i < 64; i++) {
            if(reinterpret_cast<float *>(rect.data)[i] > median) {
                binary.set(i, 1);
            }
        }
		return binary;
    }

    //! computes the hamming distance between 2 image hashes
    inline static int distance(const std::bitset<64> &hash1, const std::bitset<64> &hash2) {
        const std::bitset<64> delta = hash1 ^ hash2;
        const int distance = delta.count();
        return distance;
    }

}

