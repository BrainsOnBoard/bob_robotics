#include "common/opencv_unwrap_360.h"
#include <iostream>
#include <memory>

int
main(int argc, char **argv)
{
    // create unwrapper object for a camera of specified resolution
    cv::Size cameraResolution(1280, 400);
    OpenCVUnwrap360 unwrapper(cameraResolution);

    // open file containing unwrap params
    cv::FileStorage fs("deserialise_test.yaml", cv::FileStorage::READ);

    // read params from file
    unwrapper << fs;

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
