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
VisualNavigationBase::setMask(cv::Mat mask)
{
    // Clear mask
    if (mask.empty()) {
        m_Mask = cv::Mat{};
        return;
    }

    BOB_ASSERT(mask.size() == m_UnwrapRes);
    BOB_ASSERT(mask.type() == CV_8UC1);

    m_Mask = std::move(mask);

    // Set any non-zero element to 255 so we have a true binary mask
    std::transform(m_Mask.datastart, m_Mask.dataend, m_Mask.data,
                   [](uint8_t val) { return val ? 0xff : 0; });
}

void
VisualNavigationBase::setMask(const std::string &path)
{
    setMask(cv::imread(path, cv::IMREAD_GRAYSCALE));
}

const cv::Mat &
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
