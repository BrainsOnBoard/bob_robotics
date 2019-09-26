// BoB robotics includes
#include "imgproc/bee_eye.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace ImgProc {
namespace BeeEye {

//! Takes an input image and transforms it with the Giger bee-eye transform
void Map::getEyeView(const cv::Mat &in, cv::Mat &out)
{
    remap(in, out, m_MapX, m_MapY, cv::INTER_NEAREST);
    out.setTo(cv::Scalar{ 0, 0, 0 }, m_Mask);
}

} // BeeEye
} // ImgProc
} // BoBRobotics
