#pragma once

// OpenCV
#include "opencv2/opencv.hpp"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace ImgProc {
namespace BeeEye {

/**!
 * \brief A class for creating the bee-eye pixel maps and using them to distort
 *        an input image.
 *
 * This is based on Andy Giger's model.
 */
class Map
{
public:
    //! Create x and y pixel maps
    Map(const float eyeData[][4], const int eyeDataLength,
        const int eyeSize[2], const int imageSize[2], bool useMask = true);

    //! Takes an input image and transforms it with the Giger bee-eye transform
    void getEyeView(const cv::Mat &in, cv::Mat &out);

private:
    cv::Mat m_MapX, m_MapY, m_Mask;
}; // Map
} // BeeEye
} // ImgProc
} // BoBRobotics
