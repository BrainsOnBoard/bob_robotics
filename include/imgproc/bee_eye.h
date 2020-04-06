#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"

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
    template<size_t eyeDataLength>
    Map(const float (&eyeData)[eyeDataLength][4], const int (&eyeSize)[2],
        const int (&imageSize)[2], bool useMask = true)
    {
        cv::Size sz_out(eyeSize[0], eyeSize[1]);
        if (useMask) {
            const auto maskPath = Path::getResourcesPath() / "bee_eye_mask.png";
            m_Mask = cv::imread(maskPath.str(), cv::IMREAD_GRAYSCALE);
            BOB_ASSERT(!m_Mask.empty()); // Check file loaded successfully
            BOB_ASSERT(m_Mask.type() == CV_8UC1);
            cv::resize(m_Mask, m_Mask, sz_out);
        }

        m_MapX.create(sz_out, CV_32FC1);
        m_MapY.create(sz_out, CV_32FC1);
        for (size_t i = 0; i < eyeDataLength; i++) {
            // left eye
            m_MapX.at<float>(eyeData[i][3], 15 + eyeData[i][2]) = floor(eyeData[i][0]);
            m_MapY.at<float>(eyeData[i][3], 15 + eyeData[i][2]) = floor(eyeData[i][1]);

            // right eye
            m_MapX.at<float>(eyeData[i][3],
                             720 - 316 - eyeSize[0] - eyeData[i][2]) = imageSize[0] - floor(eyeData[i][0]);
            m_MapY.at<float>(eyeData[i][3],
                             720 - 316 - eyeSize[0] - eyeData[i][2]) = floor(eyeData[i][1]);
        }
    }

    //! Takes an input image and transforms it with the Giger bee-eye transform
    void getEyeView(const cv::Mat &in, cv::Mat &out);

private:
    cv::Mat m_MapX, m_MapY, m_Mask;
}; // Map
} // BeeEye
} // ImgProc
} // BoBRobotics
