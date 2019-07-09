#include "snapshot_processor_segment_sky.h"

#include <algorithm>

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::SnapshotProcessorArdin
//----------------------------------------------------------------------------
namespace BoBRobotics
{
namespace AntWorld
{
SnapshotProcessorSegmentSky::SnapshotProcessorSegmentSky(int outputWidth, int outputHeight)
:   m_OutputSize(outputWidth, outputHeight),
    m_IntermediateSnapshotColour(outputHeight, outputWidth, CV_8UC3),
    m_FinalSnapshot(outputHeight, outputWidth, CV_8UC1)
{
}

//----------------------------------------------------------------------------
void SnapshotProcessorSegmentSky::process(const cv::Mat &snapshot)
{
    // Resize snapshot
    cv::resize(snapshot, m_IntermediateSnapshotColour, m_OutputSize, 0.0, 0.0, cv::INTER_AREA);

    assert(m_IntermediateSnapshotColour.size() == m_FinalSnapshot.size());
    auto colourBegin = m_IntermediateSnapshotColour.begin<cv::Vec3b>();
    auto colourEnd = m_IntermediateSnapshotColour.end<cv::Vec3b>();

    auto finalBegin = m_FinalSnapshot.begin<uint8_t>();

    std::transform(colourBegin, colourEnd, finalBegin,
                   [](const cv::Vec3b &pixel)
                   {
                       return (pixel[2] == 0 && pixel[1] == 255 && pixel[0] == 255) ? 255 : 0;
                   });
}
}   // namespace AntWorld
}   // namespace BoBRobotics
