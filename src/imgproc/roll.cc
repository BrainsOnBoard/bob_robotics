// BoB robotics includes
#include "common/macros.h"
#include "imgproc/roll.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace ImgProc {

#define BOB_PIXEL_SIZE(TYPE)       \
    case cv::DataType<TYPE>::type: \
        pixelSize = sizeof(TYPE);  \
        break

void
rollLeft(const cv::Mat &in, cv::Mat &out, size_t pixelsLeft)
{
    BOB_ASSERT(in.channels() == 1);
    BOB_ASSERT(in.isContinuous());

    size_t pixelSize;
    switch (in.type()) {
        BOB_PIXEL_SIZE(uchar);
        BOB_PIXEL_SIZE(schar);
        BOB_PIXEL_SIZE(uint16_t);
        BOB_PIXEL_SIZE(int16_t);
        BOB_PIXEL_SIZE(int32_t);
        BOB_PIXEL_SIZE(float);
        BOB_PIXEL_SIZE(double);
    default:
        throw std::runtime_error("Unsupported input type");
    }

        // Make sure out is big enough
    out.create(in.size(), in.type());

    // Loop through rows
    for (int y = 0; y < in.rows; y++) {
        // Get pointer to start of row
        const uint8_t *rowPtrIn = in.ptr(y);
        uint8_t *rowPtrOut = out.ptr(y);

        // Rotate row to left by pixels
        std::rotate_copy(rowPtrIn, rowPtrIn + pixelsLeft * pixelSize,
                         rowPtrIn + in.cols * pixelSize, rowPtrOut);
    }
}

#undef BOB_CHECK_TYPE

void
rollRight(const cv::Mat &in, cv::Mat &out, size_t pixelsRight)
{
    pixelsRight %= in.cols;
    rollLeft(in, out, in.cols - pixelsRight);
}
} // ImgProc
} // BoBRobotics
