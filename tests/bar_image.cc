#include "bar_image.h"

cv::Mat
createBarImage()
{
    cv::Mat im{ cv::Size(4, 2), CV_8U };
    im = cv::Scalar(255);
    im.colRange(1, 3) = cv::Scalar(0);
    return im;
}
