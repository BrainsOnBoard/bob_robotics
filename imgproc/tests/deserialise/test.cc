// C++ includes
#include <iostream>
#include <memory>

// GeNNRobotics includes
#include "imgproc/opencv_unwrap_360.h"

int main(int argc, char **argv)
{
    // create unwrapper object for a camera of specified resolution
    cv::Size cameraResolution(1280, 400);
    cv::Size unwrapResolution(180, 50);
    GeNNRobotics::ImgProc::OpenCVUnwrap360 unwrapper(cameraResolution, unwrapResolution);

    // open file containing unwrap params
    cv::FileStorage fs("deserialise_test.yaml", cv::FileStorage::READ);

    // read params from root node of file
    fs["unwrapper"] >> unwrapper;

    // close file
    fs.release();

    // print unwrap params (in absolute pixel values)
    std::cout << "Centre: " << unwrapper.m_CentrePixel << std::endl
              << "Inner radius: " << unwrapper.m_InnerPixel << std::endl
              << "Outer radius: " << unwrapper.m_OuterPixel << std::endl
              << "Flipped: " << unwrapper.m_Flip << std::endl
              << "Offset: " << unwrapper.m_OffsetDegrees << " deg" << std::endl;

    return 0;
}
