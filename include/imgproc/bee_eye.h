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
        const int eyeSize[2], const int imageSize[2])
    {
        cv::Size sz_out(eyeSize[0], eyeSize[1]);

        m_MapX.create(sz_out, CV_32FC1);
        m_MapY.create(sz_out, CV_32FC1);
        for (int i = 0; i < eyeDataLength; i++) {
            // left eye
            m_MapX.at<float>(eyeData[i][3], 15 + eyeData[i][2]) = floor(eyeData[i][0]);
            m_MapY.at<float>(eyeData[i][3], 15 + eyeData[i][2]) = floor(eyeData[i][1]);

            // right eye
            m_MapX.at<float>(eyeData[i][3],
                             720 - 316 - eyeSize[0] - eyeData[i][2]) =
                    imageSize[0] - floor(eyeData[i][0]);
            m_MapY.at<float>(eyeData[i][3],
                             720 - 316 - eyeSize[0] - eyeData[i][2]) =
                    floor(eyeData[i][1]);
        }
    }

    //! Takes an input image and transforms it with the Giger bee-eye transform
    inline void getEyeView(const cv::Mat &in, cv::Mat &out)
    {
        remap(in, out, m_MapX, m_MapY, cv::INTER_NEAREST);
    }

private:
    cv::Mat m_MapX, m_MapY;
};
} // BeeEye
} // ImgProc
} // BoBRobotics
