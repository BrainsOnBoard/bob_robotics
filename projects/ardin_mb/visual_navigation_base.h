#pragma once

// OpenCV includes
#include <opencv2/opencv.hpp>

// Forward declarations
namespace CLI
{
class App;
}

//----------------------------------------------------------------------------
// VisualNavigationBase
//----------------------------------------------------------------------------
class VisualNavigationBase
{
public:
    virtual ~VisualNavigationBase()
    {
    }

    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) = 0;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) = 0;

    //! Perform any updates that should happen at end of test scan
    virtual void resetTestScan() = 0;

    //! Clear the memory
    virtual void clearMemory() = 0;

    virtual const cv::Size &getUnwrapResolution() const = 0;

    virtual std::pair<size_t, size_t> getHighlightedWaypoints() const = 0;

    virtual void write(cv::FileStorage&) const{}
    virtual void read(const cv::FileNode&){}

    virtual void addCLIArguments(CLI::App&){}
};
