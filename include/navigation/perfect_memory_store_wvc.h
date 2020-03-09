#pragma once

// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "differencers.h"
#include "ridf_processors.h"

// Third-party includes
#include "third_party/path.h"

// Eigen includes for matrix comparision
#include <Eigen/Dence>
#include <Eigen/Core>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cmath>

// Standard C++ includes
#include <algorithm>
#include <stdexcept>
#include <vector>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryStore {

/*numOrientations*
        ((unwrapRes.width - blockSize.width)/blockStride.width + 1)*
        ((unwrapRes.height - blockSize.height)/blockStride.height + 1);*/
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryStore::HOG
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename Differencer = AbsDiff>
class WVC
{
public:
    WVC(const cv::Size &unwrapRes, std::string wv, int level)
    {
        m_Level = level;
        m_wv = wv;
    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    size_t getNumSnapshots() const
    {
        return m_Snapshots.size();
    }

    const cv::Mat &getSnapshot(size_t) const
    {
        throw std::runtime_error("When using WVC features, snapshots aren't stored");
    }

    // Add a snapshot to memory and return its index
    size_t addSnapshot(const cv::Mat &image)
    {
        m_Snapshots.emplace_back(m_WVCDescriptorSize);
        m_WVC.compute(image, m_Snapshots.back());
        BOB_ASSERT(m_Snapshots.back().size() == m_WVCDescriptorSize);

        // Return index of new snapshot
        return (m_Snapshots.size() - 1);
    }

    void clear()
    {
        m_Snapshots.clear();
    }

    // Calculate difference between memory and snapshot with index
    float calcSnapshotDifference(const cv::Mat &image, const cv::Mat &imageMask, size_t snapshot, const cv::Mat &) const
    {
        BOB_ASSERT(imageMask.empty());

        // Calculate WVC descriptors of image
        Eigen::MatrixXd m_ReponseMatrix = m_WVC.compute(image);
        
        // Calculate differences between image WVC descriptors and snapshot
        auto diffIter = m_Differencer(m_Snapshots[snapshot], m_ResponseMatrix, m_mResponseMatrix);

        // Calculate RMS
        return Differencer::mean(std::accumulate(diffIter, diffIter, 0.0f),
                                 m_WVCDescriptorSize);
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<std::vector<float>> m_Snapshots;
    mutable Differencer m_Differencer;
    int m_Level;
    std::string m_wv;

    // This function converts a Mat Image into a 2d vector array
    vector<vector<double>>
    Image2Array(Mat matImage)
    {
        int rows = (int) matImage.rows;
        int cols = (int) matImage.cols;
        vector<vector<double>> vecImage(rows, vector<double>(cols));
        int k = 1;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                unsigned char temp;
                temp = ((uchar *) matImage.data + i * matImage.step)[j * matImage.elemSize() + k];
                vecImage[i][j] = (double) temp;
            }
        }
        return vecImage;
    }

    Mat Array2Image(vector<vector<double>> vecImage)
{
    Mat matImage(vecImage.size(), vecImage.at(0).size(), CV_64FC1);
    for(int i=0; i<matImage.rows; ++i)
    {
        for(int j=0; j<matImage.cols; ++j)
        {
            matImage.at<double>(i, j) = vecImage.at(i).at(j);
        }
          
    }
    return matImage;
}

MatrixXd Array2Matrix(vector<vector<double>> data)
{
    MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

Eigen::MatrixXd compute(cv::Mat image)
{
     // Get image dimensions
    int rows = unwrapRes.height;
    int cols = unwrapRes.width;

    vector<double> coeffs, flag;
    vector<vector<double>> vectorImage = Image2Array(image);

    // returns 1D vector that stores the output in the following format A(J) Dh(J) Dv(J) Dd(J) ..... Dh(1) Dv(1) Dd(1)
    dwt_2d(vectorImage, level, nm, coeffs, flag, length);

    vector<int> length2;
    // calculates the length of the coefficient vectors
    dwt_output_dim2(length, length2, level);
    
    // setup the new image dimensions for "display"
    int siz = length2.size();
    int rows_n = length2[siz - 2];
    int cols_n = length2[siz - 1];

    /* dwtdisp is basically a matric that arranges the coefficients in a typical 
        way if you wish to plot them like
        | LL | HL |
        | LH | HH |

    */ 
   vector<vector<double>> dwtdisp(rows_n, vector<double>(cols_n));
    dispDWT(coeffs, dwtdisp, length, length2, level);
    MatrixXd responseMatrix = Array2Matrix(dwtdisp);
}

}; // HOG
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
