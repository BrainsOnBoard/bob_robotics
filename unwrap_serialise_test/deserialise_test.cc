#include "../common/opencv_unwrap_360.h"
#include <iostream>
#include <memory>

int
main(int argc, char **argv)
{
    std::unique_ptr<OpenCVUnwrap360> unwrapper(OpenCVUnwrap360::loadFromFile(
            "yaml_read_test.yaml", cv::Size(1280, 400)));
    std::cout << "Centre: " << unwrapper->m_CentrePixel << std::endl
              << "Inner radius: " << unwrapper->m_InnerPixel << std::endl
              << "Outer radius: " << unwrapper->m_OuterPixel << std::endl
              << "Flipped: " << unwrapper->m_Flip << std::endl
              << "Offset: " << unwrapper->m_OffsetDegrees << " deg" << std::endl;

    return 0;
}
