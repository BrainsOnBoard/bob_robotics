#pragma once

// Forward declarations
namespace cv
{
    class Mat;
}

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessor
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
class SnapshotProcessor
{
public:
    //------------------------------------------------------------------------
    // Declared virtuals
    //------------------------------------------------------------------------
    // Process input snapshot (probably at screen resolution)
    virtual void process(const cv::Mat &snapshot) = 0;

    // Get processed snapshot - should be in a CV_8UC1 format image
    virtual const cv::Mat &getFinalSnapshot() const = 0;
};
}   // namespace AntWorld
}   // namespace BoBRobotics
