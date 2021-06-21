// BoB robotics includes
#include "common/macros.h"
#include "imgproc/mask.h"
#include "imgproc/roll.h"

// Standard C++ includes
#include <stdexcept>

namespace BoBRobotics {
namespace ImgProc {

Mask::Mask(cv::Mat mask, const cv::Size &resizeTo)
{
    set(std::move(mask), resizeTo);
}

Mask::Mask(const filesystem::path &imagePath, const cv::Size &resizeTo)
{
    set(imagePath, resizeTo);
}

void
Mask::apply(const cv::Mat &in, cv::Mat &out) const
{
    if (empty()) {
        out = in;
    } else {
        cv::bitwise_and(in, m_Mask, out);
    }
}

void
Mask::combine(const Mask &otherMask, Mask &outputMask) const
{
    if (empty()) {
        outputMask = otherMask;
    } else if (otherMask.empty()) {
        outputMask = *this;
    } else {
        cv::bitwise_and(m_Mask, otherMask.get(), outputMask.m_Mask);
    }
}

size_t
Mask::countUnmaskedPixels(const cv::Size &size) const
{
    return empty() ? size.width * size.height : cv::countNonZero(m_Mask);
}

bool Mask::empty() const
{
    return m_Mask.empty();
}

const cv::Mat &
Mask::get() const
{
    return m_Mask;
}

bool
Mask::isValid(const cv::Size &imageSize) const
{
    return empty() || m_Mask.size() == imageSize;
}

void
Mask::roll(Mask &out, size_t pixelsLeft) const
{
    if (empty()) {
        out.m_Mask = cv::Mat{};
    } else {
        ImgProc::roll(m_Mask, out.m_Mask, pixelsLeft);
    }
}

void
Mask::set(cv::Mat mask, const cv::Size &resizeTo)
{
    if (mask.empty()) {
        // Clears mask
        m_Mask = cv::Mat{};
        return;
    }

    // The user has requested a specific size of mask
    if (resizeTo != cv::Size{ 0, 0 }) {
        cv::resize(mask, mask, resizeTo, {}, {}, cv::INTER_NEAREST);
    }

    // Needs to be composed of bytes
    mask.convertTo(mask, CV_8UC1);

    /*
     * Make into true binary mask. This is more convenient and also means a mask
     * can be applied with a bitwise-and operation.
     */
    cv::threshold(mask, mask, 127.0, 255.0, cv::THRESH_BINARY);

    m_Mask = std::move(mask);
}

void
Mask::set(const filesystem::path &imagePath, const cv::Size &resizeTo)
{
    cv::Mat mask = cv::imread(imagePath.str(), cv::IMREAD_GRAYSCALE);
    if (mask.empty()) {
        throw std::runtime_error("Could not load mask from " + imagePath.str());
    }

    set(std::move(mask), resizeTo);
}

} //ImgProc
} // BoBRobotics
