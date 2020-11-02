#pragma once

// BoB robotics includes
#include "common/macros.h"

// Third-party includes
#include "third_party/path.h"

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace ImgProc {
template<class T = uchar>
class Mask {
public:
    Mask(const cv::Mat &mask)
      : m_GoodPixels(maskToVector(mask))
      , m_Size(mask.size())
    {
    }

    Mask(const filesystem::path &path)
      : Mask(cv::imread(path.str(), cv::IMREAD_GRAYSCALE))
    {
    }

    auto &operator()(const cv::Mat &image) const
    {
        BOB_ASSERT(image.type() == cv::DataType<T>::type);
        BOB_ASSERT(image.size() == m_Size);

        // Get selected pixels from image
        m_Output.resize(m_GoodPixels.size());
        const auto getPixel = [&image](size_t pixel) {
            return reinterpret_cast<T *>(image.data)[pixel];
        };
        std::transform(m_GoodPixels.cbegin(), m_GoodPixels.cend(),
                       m_Output.begin(), getPixel);

        return m_Output;
    }

    cv::Mat reconstructImage(const std::vector<T> &maskedImage) const
    {
        BOB_ASSERT(maskedImage.size() == m_GoodPixels.size());

        // cv::Mats apparently aren't automatically zeroed, so do it explicitly
        cv::Mat out = cv::Mat::zeros(m_Size, cv::DataType<T>::type);
        auto imageIt = maskedImage.cbegin();
        for (size_t pixel : m_GoodPixels) {
            reinterpret_cast<T *>(out.data)[pixel] = *imageIt++;
        }

        return out;
    }

    cv::Size size() const { return m_Size; }

private:
    const std::vector<size_t> m_GoodPixels;
    mutable std::vector<T> m_Output;
    const cv::Size m_Size;

    static std::vector<size_t> maskToVector(const cv::Mat &mask)
    {
        BOB_ASSERT(!mask.empty());
        BOB_ASSERT(mask.type() == CV_8UC1);

        // Track which pixels are the good ones
        std::vector<size_t> goodPixels;
        for (size_t i = 0; i < mask.total(); i++) {
            if (mask.data[i]) {
                goodPixels.push_back(i);
            }
        }

        return goodPixels;
    }
}; // Mask
} // ImgProc
} // BoBRobotics
