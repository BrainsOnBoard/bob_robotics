// BoB robotics includes
#include "common/macros.h"
#include "navigation/visual_navigation_base.h"

// Third-party includes
#include "plog/Log.h"

#ifdef _OPENMP
// OpenMP
#include <omp.h>
#endif

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
namespace Navigation {

int getMaxThreads()
{
#ifdef _OPENMP
    return omp_get_max_threads();
#else
    return 1;
#endif
}

VisualNavigationBase::VisualNavigationBase(const cv::Size &unwrapRes)
  : m_UnwrapRes(unwrapRes)
{
#ifdef _OPENMP
    LOGI << "Running on " << getMaxThreads() << " threads" << std::endl;
#else
    LOGW << "This program was not compiled with OpenMP support. Execution will be single threaded." << std::endl;
#endif
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

void
VisualNavigationBase::setMask(ImgProc::Mask mask)
{
    m_Mask = std::move(mask);
}

const ImgProc::Mask &
VisualNavigationBase::getMask() const
{
    return m_Mask;
}

const cv::Size &
VisualNavigationBase::getUnwrapResolution() const
{
    return m_UnwrapRes;
}

} // Navigation
} // BoBRobotics
