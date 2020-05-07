#pragma once

// Forward declarations
namespace cv
{
    class Mat;
}


namespace BoBRobotics {
namespace StoneCX {
void visualize(cv::Mat &activityImage, const float *rTL, const float *rCL1, const float *rTB1, const float *rTN2,
               const float *rCPU4, const float *rPontine, const float *rCPU1);
}
}
