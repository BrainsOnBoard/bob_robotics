// BoB robotics includes
#include "common/macros.h"
#include "imgproc/convert_scale.h"

// Standard C++ includes
#include <limits>
#include <stdexcept>
#include <type_traits>

// Standard C includes
#include <cstdint>

// Anonymous namespace
namespace {
template<class T, typename = void>
struct ImageMax;

template<class T>
struct ImageMax<T, std::enable_if_t<std::is_unsigned<T>::value>> {
    static constexpr T value = std::numeric_limits<T>::max();
};

template<class T>
struct ImageMax<T, std::enable_if_t<std::is_floating_point<T>::value>> {
    static constexpr T value{ 1 };
};

#define BOB_GET_MAX(TYPE)          \
    case cv::DataType<TYPE>::type: \
        return ImageMax<TYPE>::value

inline double
getMax(int type)
{
    switch (type) {
        BOB_GET_MAX(uchar);
        BOB_GET_MAX(uint16_t);
        BOB_GET_MAX(float);
        BOB_GET_MAX(double);
    default:
        throw std::invalid_argument("Unsupported matrix type");
    }
}
}

namespace BoBRobotics {
namespace ImgProc {
void
convertScale(const cv::Mat &in, cv::Mat &out, int targetType)
{
    BOB_ASSERT(in.channels() == 1);

    if (in.type() == targetType) {
        out = in;
        return;
    }

    in.convertTo(out, targetType, getMax(targetType) / getMax(in.type()));
}

} // ImgProc
} // BoBRobotics
