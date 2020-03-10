#include "antworld/snapshot_processor_gray.h"
#include <algorithm>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessorGray
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
SnapshotProcessorGray::SnapshotProcessorGray(int outputWidth, int outputHeight)
:   m_OutputSize(outputWidth, outputHeight),
    m_IntermediateSnapshotColour(outputHeight, outputWidth, CV_8UC3),
    m_FinalSnapshot(outputHeight, outputWidth, CV_8UC1)
{
}

//----------------------------------------------------------------------------
void SnapshotProcessorGray::process(const cv::Mat &snapshot)
{
    // Resize snapshot
    cv::resize(snapshot, m_IntermediateSnapshotColour, m_OutputSize, 0.0, 0.0, cv::INTER_AREA);

    cv::Mat graySnapshot;
    cv::cvtColor(snapshot, m_FinalSnapshot, cv::COLOR_BGR2GRAY);

    
    
}
}   // namespace AntWorld
}   // namespace BoBRobotics