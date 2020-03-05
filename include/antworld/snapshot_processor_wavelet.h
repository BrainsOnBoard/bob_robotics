#pragma once

// OpenCV includes
#include "third_party/wavelet2s/wavelet2s.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace BoBRobotics
{
namespace AntWorld
{
//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessor
//----------------------------------------------------------------------------
//! OpenCV-based snapshot processor - uses OpenCV on CPU to process snapshots
class SnapshotProcessorWavelet
{
public:
    SnapshotProcessorWavelet(int displayScale, int intermediateWidth, int intermediateHeight,
                           int outputWidth, int outputHeight, std::string waveletName, int level);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // Process input snapshot (probably at screen resolution)
    void process(const cv::Mat &snapshot);
    void TransferToWaveletDomain(cv::Mat img, int level, std::string nm);
    Eigen::MatrixXd selectResponses(Eigen::MatrixXd responseMatrix, std::string selectionMode);
    std::vector<std::vector<double>> Image2Array(cv::Mat matImage);
    Eigen::MatrixXd Array2Matrix(std::vector<std::vector<double>> data);
    void calcFilterResponses(std::vector<std::vector<double>> vectorImage, int level,std::string nm, Eigen::MatrixXd &M);
    
    
    const cv::Mat &getFinalSnapshot() const{ return m_FinalSnapshot; }
    const cv::Mat &getFinalSnapshotFloat() const{ return m_FinalSnapshotFloat; }

private:
    //------------------------------------------------------------------------
    // Private members
    //------------------------------------------------------------------------
    // How much larger than intermediate image size is snapshot
    const int m_DisplayScale;

    // Dimensions of intermediate image
    const int m_IntermediateWidth;
    const int m_IntermediateHeight;

    // Dimensions of final output
    const int m_OutputWidth;
    const int m_OutputHeight;

    std::string m_WaveletName;
    int m_Level;

    // Containers for filter coeffs and flags 
    std::vector<double> m_Coeffs;
    std::vector<double> m_Flag;

    // Host OpenCV array to hold intermediate resolution greyscale snapshot
    cv::Mat m_IntermediateSnapshotGreyscale;

    // Host OpenCV array to hold final resolution greyscale snapshot
    cv::Mat m_FinalSnapshot;
    cv::Mat m_FinalSnapshotFloat;

    // Host 2d vector array for interfacing with wavelet library
    std::vector<std::vector<double>> m_VectorSnapshot;

    // Host eigen matrix for convinient filter response storage
    Eigen::MatrixXd m_ResponseMatrix;
    
    // CLAHE algorithm for histogram normalization
    cv::Ptr<cv::CLAHE> m_Clahe;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
