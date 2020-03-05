// BoB robotics includes
#include "antworld/snapshot_processor_wavelet.h"
#include "common/macros.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessorWavelet
//----------------------------------------------------------------------------
namespace BoBRobotics 
{
namespace AntWorld 
{
SnapshotProcessorWavelet::SnapshotProcessorWavelet(int displayScale, int intermediateWidth, int intermediateHeight, int outputWidth, int outputHeight, const std::string waveletName, const int level)
  : m_DisplayScale(displayScale)
  , m_IntermediateWidth(intermediateWidth)
  , m_IntermediateHeight(intermediateHeight)
  , m_OutputWidth(outputWidth)
  , m_OutputHeight(outputHeight)
  , m_IntermediateSnapshotGreyscale(intermediateHeight, intermediateWidth, CV_8UC1)
  , m_FinalSnapshot(outputHeight, outputWidth, CV_8UC1)
  , m_FinalSnapshotFloat(outputHeight, outputWidth, CV_32FC1)
  ,  m_Clahe(cv::createCLAHE(40.0, cv::Size(8, 8)))
            
{
    // Check that the display scale is a multiple of 4
    BOB_ASSERT(m_DisplayScale % 4 == 0);
    m_Level = level;
    m_WaveletName = waveletName;
    m_ResponseMatrix = Eigen::MatrixXd(outputHeight,outputWidth);
    std::vector<std::vector<double>> m_VectorSnapshot(outputHeight, std::vector<double>(outputWidth,1)); 

}
//----------------------------------------------------------------------------
void
SnapshotProcessorWavelet::process(const cv::Mat &snapshot)
{
    // Check snapshot is expected size
    BOB_ASSERT(snapshot.rows == m_IntermediateHeight * m_DisplayScale);
    BOB_ASSERT(snapshot.cols == m_IntermediateWidth * m_DisplayScale);

    // Calculate start and end offset of resize kernel
    const int kernelStart = m_DisplayScale / 4;
    const int kernelEnd = m_DisplayScale - kernelStart;
    const int kernelPixels = kernelStart * m_DisplayScale;

    // Loop through intermediate image rows
    // **NOTE** this technique for downsampling the image is taken from the Matlab
    // and MASSIVELY improves performance over standard cv::imresize algorithms
    for (int y = 0; y < m_IntermediateHeight; y++) {
        // Calculate rows averaging kernel should operate over
        const int kernelStartY = (y * m_DisplayScale) + kernelStart;
        const int kernelEndY = (y * m_DisplayScale) + kernelEnd;

        // Loop through intermediate image columns
        for (int x = 0; x < m_IntermediateWidth; x++) {
            // Calculate columns averaging kernel should operate over
            const int kernelStartX = (x * m_DisplayScale) + kernelStart;
            const int kernelEndX = (x * m_DisplayScale) + kernelEnd;

            // Loop over snapshot pixels in kernel and sum their green components
            int sum = 0;
            for (int i = kernelStartY; i < kernelEndY; i++) {
                for (int j = kernelStartX; j < kernelEndX; j++) {
                    sum += snapshot.at<cv::Vec3b>(i, j)[1];
                }
            }

            // Divide sum by number of pixels in kernel to compute average and write to intermediate snapshot
            m_IntermediateSnapshotGreyscale.at<uint8_t>(y, x) = static_cast<uint8_t>(sum / kernelPixels);
        }
    }

    // Invert image
    cv::subtract(255, m_IntermediateSnapshotGreyscale, m_IntermediateSnapshotGreyscale);

    // Apply histogram normalization
    // http://answers.opencv.org/question/15442/difference-of-clahe-between-opencv-and-matlab/
    m_Clahe->apply(m_IntermediateSnapshotGreyscale, m_IntermediateSnapshotGreyscale);

    // Finally resample down to final size
    cv::resize(m_IntermediateSnapshotGreyscale, m_FinalSnapshot, cv::Size(m_OutputWidth, m_OutputHeight), 0.0, 0.0, cv::INTER_CUBIC);

    // take the final image and transfer it to 
    SnapshotProcessorWavelet::TransferToWaveletDomain(m_FinalSnapshot, m_Level, m_WaveletName);

}

void 
SnapshotProcessorWavelet::TransferToWaveletDomain(cv::Mat img,
                                                  int level,
                                                  std::string nm)
{
    // transfer image into wavelib comparable representation
    SnapshotProcessorWavelet::Image2Array(img, m_VectorSnapshot);

    // get filter responses in matrix format
    SnapshotProcessorWavelet::calcFilterResponses(m_VectorSnapshot, level, nm, m_ResponseMatrix);

    // select relevant filter responses
    std::string selectionMode = "normalize";
    m_ResponseMatrix = SnapshotProcessorWavelet::selectResponses(m_ResponseMatrix, selectionMode);

    // cast back to cv format 
    cv::eigen2cv(m_ResponseMatrix,m_FinalSnapshot);
    cv::eigen2cv(m_ResponseMatrix,m_FinalSnapshotFloat);
    
}

void SnapshotProcessorWavelet::Image2Array(cv::Mat matImage, std::vector<std::vector<double>> &vecImage)
{
    int rows = (int) matImage.rows;
    int cols = (int) matImage.cols;
    int k = 1;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            unsigned char temp;
            temp = ((uchar *) matImage.data + i * matImage.step)[j * matImage.elemSize() + k];
            vecImage[i][j] = (double) temp;
        }
    }
}

void SnapshotProcessorWavelet::calcFilterResponses(std::vector<std::vector<double>> vectorImage,
                                              int level,
                                              std::string nm,
                                              Eigen::MatrixXd &M)
{
    std::vector<double> coeffs;
    std::vector<double> flag;
    std::vector<int> length;
    // returns 1D vector that stores the output in the following format A(J) Dh(J) Dv(J) Dd(J) ..... Dh(1) Dv(1) Dd(1)
    WAVELET2S_H::dwt_2d(vectorImage, level, nm, coeffs, flag, length);

    std::vector<int> length2;
    // calculates the length of the coefficient vectors
    WAVELET2S_H::dwt_output_dim2(length, length2, level);

    // setup the new image dimensions for display
    int siz = length2.size();
    int rows_n = length2[siz - 2];
    int cols_n = length2[siz - 1];

    /* dwtdisp is basically a matrix that arranges the coefficients in a typical 
        way if you wish to plot them like
        | LL | HL |
        | LH | HH |

    */
    // create container structure for coefficients
    std::vector<std::vector<double>> dwtdisp(rows_n, vector<double>(cols_n));
    WAVELET2S_H::dispDWT(coeffs, dwtdisp, length, length2, level);

    // recast into more convinient storage container
    M = Array2Matrix(dwtdisp);
}

Eigen::MatrixXd SnapshotProcessorWavelet::Array2Matrix(std::vector<std::vector<double>> data)
{
    Eigen::MatrixXd eMatrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); ++i)
        eMatrix.row(i) = Eigen::VectorXd::Map(&data[i][0], data[0].size());
    return eMatrix;
}

Eigen::MatrixXd
SnapshotProcessorWavelet::selectResponses(Eigen::MatrixXd responseMatrix, std::string selectionMode)
{
    if (selectionMode.compare((std::string)"vertical"))
        {

            Eigen::MatrixXd sub = responseMatrix;
            double maxValue = sub.maxCoeff();
            for (int i = 0; i < sub.rows(); i++) {
                for (int j = 0; j < sub.cols(); j++) {
                    if (sub(i, j) <= 0) {
                        sub(i, j) = 0;
                    } else {
                        sub(i, j) = sub(i, j) / maxValue;
                    }
                }
            }

            return sub;
        }
    else if (selectionMode.compare((std::string) "normalize"))
    {
            Eigen::MatrixXd sub = responseMatrix;
            double maxValue = sub.maxCoeff();
            for (int i = 0; i < sub.rows(); i++) {
                for (int j = 0; j < sub.cols(); j++) {
                    if (sub(i, j) <= 0) {
                        sub(i, j) = 0;
                    } else {
                        sub(i, j) = sub(i, j) / maxValue;
                    }
                }
            }

            return sub;
    }
    else 
    {
        return responseMatrix;
    }
}

} // namespace AntWorld
} // namespace BoBRobotics
