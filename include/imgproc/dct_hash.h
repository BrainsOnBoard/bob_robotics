#pragma once

// Standard C++ includes
#include <iostream>
#include <string>
#include <algorithm>

// Standard C includes
#include <cmath>

//opencv
#include <opencv2/opencv.hpp>


class DCTHash {

    private:
    static unsigned int countSetBits(unsigned long long int n) { 
        unsigned int count = 0; 
        while (n) { 
            count += n & 1; 
            n >>= 1; 
        } 
        return count; 
    } 

    public:
    
    
    //! computes the DCT hash 
    static unsigned long long computeHash(const cv::Mat &in) {
        
		unsigned long long hash;
        cv::Mat dct_mat;
        cv::dct(in, dct_mat);
        cv::Rect roi( 0, 0, 8, 8); 
        cv::Mat low_dct( dct_mat, roi ); 

		std::vector<float> vec;
		for (int row = 0; row < 9; row++) {
			for (int col = 0; col < 9; col++) {
			vec.push_back(low_dct.at<float>(row,col));
			}
		}

		// calculate median value 
		std::sort(std::begin(vec), std::end(vec));
		float medianv;
		int n = vec.size();
		if (n % 2 != 0) {
			medianv = vec[n/2];
		} else {
			medianv = (vec[(n-1)/2] + vec[n/2]) / 2;
		}
		
		
		const cv::Mat mask = (low_dct > medianv);
		const cv::Mat row = mask.reshape(1,1)/255;

		std::vector<int> hashvector;
		row.row(0).copyTo(hashvector);
		std::stringstream ss;
		std::string binstring;
		for (int v : hashvector) { ss << v; }
		ss >> binstring;
		unsigned long long int binary = std::stoull(binstring, 0, 2);
		hash = binary;
		return hash;

    }

    //! computes the hamming distance between 2 image hashes
    static int distance(const unsigned long long &hash1, const unsigned long long int &hash2) {
        const unsigned long long int xorhash = hash1 ^ hash2;
        const int difference_bits = countSetBits(xorhash);
        return difference_bits;
    }



};

