#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>

namespace BoBRobotics {
namespace ImgProc {
//----------------------------------------------------------------------------
// OpenCVOpticalFlow
//----------------------------------------------------------------------------
class OpenCVOpticalFlow
{
public:
    OpenCVOpticalFlow();

    OpenCVOpticalFlow(const cv::Size &inputRes);

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void create(const cv::Size &inputRes);

    bool calculate(const cv::Mat &input);

    void render(cv::Mat &outputImage, int scale);

    const cv::Mat &getFlowX() const;
    const cv::Mat &getFlowY() const;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    cv::Mat m_Frames[2];
    unsigned int m_Frame;

    cv::Ptr<cv::superres::FarnebackOpticalFlow> m_OpticalFlow = cv::superres::createOptFlow_Farneback();
    cv::Mat m_FlowX;
    cv::Mat m_FlowY;
}; // OpenCVOpticalFlow
}  // ImgProc
}  // BoBRobotics
