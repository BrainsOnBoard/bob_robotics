#pragma once

// BoB robotics includes
#include "common/logging.h"
#include "common/macros.h"
#include "differencers.h"
#include "ridf_processors.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/wavelet2s/wavelet2s.h"

// Eigen includes for matrix comparision
#include <Eigen/Dense>
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

namespace BoBRobotics {
namespace Navigation {
namespace PerfectMemoryStore {

/*numOrientations*
        ((unwrapRes.width - blockSize.width)/blockStride.width + 1)*
        ((unwrapRes.height - blockSize.height)/blockStride.height + 1);*/
//------------------------------------------------------------------------
// BoBRobotics::Navigation::PerfectMemoryStore::WVC
//------------------------------------------------------------------------
//! Perfect memory algorithm using HOG features instead of raw image matching
template<typename Differencer = AbsDiff>
class WVC
{
public:
    WVC(const cv::Size &unwrapRes, std::string wv, int level)
    : m_Differencer(unwrapRes.height*unwrapRes.width)
    {
        m_UnwrapRes = unwrapRes;
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
        m_Snapshots.emplace_back((m_UnwrapRes.height*m_UnwrapRes.width));
        compute(image, m_Snapshots.back());
        
        
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
        std::vector<double>  m_responseVector;
        compute(image, m_responseVector);

        // Calculate differences between image WVC descriptors and snapshot
        auto diffIter = m_Differencer(m_Snapshots[snapshot], m_responseVector, m_responseVector);

        // Calculate RMS
        return Differencer::mean(std::accumulate(diffIter, diffIter, 0.0f),
                                 (m_UnwrapRes.height*m_UnwrapRes.width));
    }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    std::vector<std::vector<double>> m_Snapshots;
    mutable Differencer m_Differencer;
    int m_Level;
    std::string m_wv;
    cv::Size m_UnwrapRes;

    // This function converts a Mat Image into a 2d vector array
    std::vector<std::vector<double>>
    Image2Array(cv::Mat matImage) const
    {
        int rows = (int) matImage.rows;
        int cols = (int) matImage.cols;
        std::vector<std::vector<double>> vecImage(rows, std::vector<double>(cols));
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

    cv::Mat Array2Image(std::vector<std::vector<double>> vecImage) const
{
    cv::Mat matImage(vecImage.size(), vecImage.at(0).size(), CV_64FC1);
    for(int i=0; i<matImage.rows; ++i)
    {
        for(int j=0; j<matImage.cols; ++j)
        {
            matImage.at<double>(i, j) = vecImage.at(i).at(j);
        }
          
    }
    return matImage;
}

Eigen::MatrixXd Array2Matrix(std::vector<std::vector<double>> data) const
{
    Eigen::MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

void compute(cv::Mat image, vector<double>  &responseVector)const
{
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image);
    cv::waitKey(0);
    Eigen::MatrixXd responseMatrix;
    Eigen::VectorXd rVec;
    std::vector<double> coeffs, flag;
    std::vector<std::vector<double>> vectorImage = Image2Array(image);

    // returns 1D vector that stores the output in the following format A(J) Dh(J) Dv(J) Dd(J) ..... Dh(1) Dv(1) Dd(1)
    std::vector<int> length;
    dwt_2d(vectorImage, m_Level, m_wv, coeffs, flag, length);

    std::vector<int> length2;
    // calculates the length of the coefficient vectors
    dwt_output_dim2(length, length2, m_Level);
    
    // setup the new image dimensions for "display"
    int siz = length2.size();
    int rows_n = length2[siz - 2];
    int cols_n = length2[siz - 1];

    /* dwtdisp is basically a matric that arranges the coefficients in a typical 
        way if you wish to plot them like
        | LL | HL |
        | LH | HH |

    */ 
   std::vector<std::vector<double>> dwtdisp(rows_n, std::vector<double>(cols_n));
    dispDWT(coeffs, dwtdisp, length, length2, m_Level);
    responseMatrix = Array2Matrix(dwtdisp);
    rVec = Eigen::Map<const Eigen::VectorXd>(responseMatrix.data(), responseMatrix.size());
    
    cv::Mat subImage;
    cv::eigen2cv(responseMatrix,subImage);
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", subImage);
    cv::waitKey(0);
    // cast to vector<double>
    responseVector.resize(rVec.size());
    Eigen::VectorXd::Map(&responseVector[0], rVec.size()) = rVec;
    
}

}; // HOG
} // PerfectMemoryStore
} // Navigation
} // BoBRobotics
