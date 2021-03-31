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

class DCTHash {

    public:
    //! computes the DCT hash
    static std::bitset<64> computeHash(const cv::Mat &in) {

        cv::Mat dct_mat;
        cv::dct(in, dct_mat);
        cv::Rect roi( 0, 0, 8, 8);
        cv::Mat low_dct( dct_mat, roi );

		std::vector<float> vec;
		for (int row = 0; row < 8; row++) {
			for (int col = 0; col < 8; col++) {
			    vec.push_back(low_dct.at<float>(row,col));
			}
		}

		// calculate median value
		std::sort(std::begin(vec), std::end(vec));
		float medianv;
		const int n = 64;
		if (n % 2 != 0) {
			medianv = vec[n/2];
		} else {
			medianv = (vec[(n-1)/2] + vec[n/2]) / 2;
		}

        std::bitset<64> binary;
        for (size_t i = 0; i < 64; i++) {
            if(reinterpret_cast<float *>(low_dct.data)[i] > medianv) {
                binary.set(i, 1);
            }
        }
		return binary;
    }

    //! computes the hamming distance between 2 image hashes
    static int distance(const std::bitset<64> &hash1, const std::bitset<64> &hash2) {
        const std::bitset<64> delta = hash1 ^ hash2;
        const int distance = delta.count();
        return distance;
    }

};

