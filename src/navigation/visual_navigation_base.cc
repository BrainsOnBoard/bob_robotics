// BoB robotics includes
#include "navigation/visual_navigation_base.h"
#include "common/macros.h"

namespace BoBRobotics {
namespace Navigation {

VisualNavigationBase::VisualNavigationBase(const cv::Size &unwrapRes)
  : m_UnwrapRes(unwrapRes)
{}

VisualNavigationBase::~VisualNavigationBase()
{}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
void
VisualNavigationBase::trainRoute(const ImageDatabase &imdb, bool resizeImages)
{
    cv::Mat image;
    if (resizeImages) {
        cv::Mat imageResized;
        for (const auto &e : imdb) {
            image = e.loadGreyscale();
            BOB_ASSERT(image.type() == CV_8UC1);
            cv::resize(image, imageResized, m_UnwrapRes);
            train(imageResized);
        }
    } else {
        for (const auto &e : imdb) {
            image = e.loadGreyscale();
            BOB_ASSERT(image.type() == CV_8UC1);
            BOB_ASSERT(image.cols == m_UnwrapRes.width);
            BOB_ASSERT(image.rows == m_UnwrapRes.height);
            train(image);
        }
    }
}

void
VisualNavigationBase::setMaskImage(const std::string &path)
{
    m_MaskImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
    BOB_ASSERT(m_MaskImage.cols == m_UnwrapRes.width);
    BOB_ASSERT(m_MaskImage.rows == m_UnwrapRes.height);
    BOB_ASSERT(m_MaskImage.type() == CV_8UC1);
}

const cv::Mat &
VisualNavigationBase::getMaskImage() const
{
    return m_MaskImage;
}

const cv::Size &
VisualNavigationBase::getUnwrapResolution() const
{
    return m_UnwrapRes;
}

} // Navigation
} // BoBRobotics
