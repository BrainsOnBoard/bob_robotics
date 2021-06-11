// BoB robotics includes
#include "common/macros.h"
#include "navigation/visual_navigation_base.h"

// Third-party includes
#include "plog/Log.h"

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Navigation {

VisualNavigationBase::VisualNavigationBase(const cv::Size &unwrapRes)
  : m_UnwrapRes(unwrapRes)
{
}

VisualNavigationBase::~VisualNavigationBase()
{}

//------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------
void
VisualNavigationBase::trainRoute(const ImageDatabase &imdb, bool resizeImages)
{
    trainRoute(imdb.loadImages(resizeImages ? m_UnwrapRes : cv::Size{}));
}

void
VisualNavigationBase::trainRoute(const std::vector<cv::Mat> &images)
{
    for (auto &image : images) {
        train(image);
    }
}

const cv::Size &
VisualNavigationBase::getUnwrapResolution() const
{
    return m_UnwrapRes;
}

} // Navigation
} // BoBRobotics
