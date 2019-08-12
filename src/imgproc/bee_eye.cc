// BoB robotics includes
#include "common/bob_robotics_path.h"
#include "common/macros.h"
#include "imgproc/bee_eye.h"

// Standard C includes
#include <cmath>

namespace BoBRobotics {
namespace ImgProc {
namespace BeeEye {

Map::Map(const float eyeData[][4], const int eyeDataLength,
         const int eyeSize[2], const int imageSize[2], bool useMask)
{
    cv::Size sz_out(eyeSize[0], eyeSize[1]);
    if (useMask) {
        const auto maskPath = getResourcesPath() / "bee_eye_mask.png";
        m_Mask = cv::imread(maskPath.str(), cv::IMREAD_GRAYSCALE);
        BOB_ASSERT(m_Mask.type() == CV_8UC1);
        cv::resize(m_Mask, m_Mask, sz_out);
    }

    m_MapX.create(sz_out, CV_32FC1);
    m_MapY.create(sz_out, CV_32FC1);
    for (int i = 0; i < eyeDataLength; i++) {
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
void Map::getEyeView(const cv::Mat &in, cv::Mat &out)
{
    remap(in, out, m_MapX, m_MapY, cv::INTER_NEAREST);
    out.setTo(cv::Scalar{ 0, 0, 0 }, m_Mask);
}

} // BeeEye
} // ImgProc
} // BoBRobotics
