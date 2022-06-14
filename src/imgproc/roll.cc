// BoB robotics includes
#include "common/macros.h"
#include "imgproc/roll.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace ImgProc {
void
rollLeft(const cv::Mat &in, cv::Mat &out, size_t pixelsLeft)
{
    BOB_ASSERT(in.type() == CV_8UC1);

    // Make sure out is big enough
    out.create(in.size(), CV_8UC1);

    // Loop through rows
    for (int y = 0; y < in.rows; y++) {
        // Get pointer to start of row
        const uint8_t *rowPtrIn = in.ptr(y);
        uint8_t *rowPtrOut = out.ptr(y);

        // Rotate row to left by pixels
        std::rotate_copy(rowPtrIn, rowPtrIn + pixelsLeft, rowPtrIn + in.cols, rowPtrOut);
    }
}

void
rollRight(const cv::Mat &in, cv::Mat &out, size_t pixelsRight)
{
    pixelsRight %= in.cols;
    rollLeft(in, out, in.cols - pixelsRight);
}
} // ImgProc
} // BoBRobotics
