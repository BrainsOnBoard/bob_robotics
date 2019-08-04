#define NO_HEADER_DEFINITIONS
#include "image_input.h"

// BoB robotics includes
#include "common/logging.h"

// Snapshot bot includes
#include "config.h"

using namespace BoBRobotics;

//----------------------------------------------------------------------------
// ImageInput
//----------------------------------------------------------------------------
ImageInput::ImageInput(const Config &config)
:   m_InputSize(config.getUnwrapRes())
{
}


//----------------------------------------------------------------------------
// ImageInputRaw
//----------------------------------------------------------------------------
ImageInputRaw::ImageInputRaw(const Config &config)
:   ImageInput(config), m_GreyscaleUnwrapped(config.getUnwrapRes(), CV_8UC1)
{
}
//----------------------------------------------------------------------------
const cv::Mat &ImageInputRaw::processSnapshot(const cv::Mat &snapshot)
{
    // Convert snapshot to greyscale and return
    cv::cvtColor(snapshot, m_GreyscaleUnwrapped, cv::COLOR_BGR2GRAY);
    return m_GreyscaleUnwrapped;
}

//----------------------------------------------------------------------------
// ImageInputBinary
//----------------------------------------------------------------------------
ImageInputBinary::ImageInputBinary(const Config &config)
:   ImageInput(config), m_SegmentIndices(config.getUnwrapRes(), CV_32SC1), 
    m_SegmentedImage(config.getUnwrapRes().height - 2, config.getUnwrapRes().width - 2, CV_8UC1)
{
    // Read marker image
    // **NOTE** will read 8-bit per channel grayscale
    m_MarkerImage = cv::imread(config.getWatershedMarkerImageFilename(), cv::IMREAD_GRAYSCALE);
    BOB_ASSERT(m_MarkerImage.size() == config.getUnwrapRes());

    // Convert marker image to 32-bit per channel without performing any scaling
    m_MarkerImage.convertTo(m_MarkerImage, CV_32SC1);

    // Find minimum and maximum elements
    const auto minMaxIter = std::minmax_element(m_MarkerImage.begin<int32_t>(), m_MarkerImage.end<int32_t>());
    m_MinIndex = *minMaxIter.first;
    m_MaxIndex = *minMaxIter.second;

    // Check that there are indeed 3 markers as expected
    BOB_ASSERT(m_MinIndex == 0);
    BOB_ASSERT(m_MaxIndex == 2);
}
//----------------------------------------------------------------------------
const cv::Mat &ImageInputBinary::processSnapshot(const cv::Mat &snapshot)
{
    // Read indices of segments
    const cv::Mat segmentedIndices = readSegmentIndices(snapshot);

    // Convert to greyscale image
    // **NOTE** segmentedIndices should contain -1, 1 and 2 - this rescales to black, white and grey
    segmentedIndices.convertTo(m_SegmentedImage, CV_8UC1, 85.0, 85.0);
    BOB_ASSERT(m_SegmentedImage.size() == getOutputSize());
    return m_SegmentedImage;
}
//----------------------------------------------------------------------------
cv::Mat ImageInputBinary::readSegmentIndices(const cv::Mat &snapshot)
{
    // Make a copy of marker image to perform segmentation on
    m_MarkerImage.copyTo(m_SegmentIndices);

    // Segment!
    cv::watershed(snapshot, m_SegmentIndices);

    // For some reason watershed thresholding results in a border around image so return ROI inside this
    // **NOTE** we don't use getOutputSize() here as it may be overriden in derived classes
    return cv::Mat(m_SegmentIndices, cv::Rect(1, 1, getInputSize().width - 2, getInputSize().height - 2));
}

//----------------------------------------------------------------------------
// ImageInputHorizon
//----------------------------------------------------------------------------
ImageInputHorizon::ImageInputHorizon(const Config &config)
:   ImageInputBinary(config), m_HorizonVector(1, config.getUnwrapRes().width - 2, CV_8UC1),
    m_ColumnHorizonPixelsSum(config.getUnwrapRes().width - 2), m_ColumnHorizonPixelsCount(config.getUnwrapRes().width - 2)
{
    // Check image will be representable as 8-bit value
    BOB_ASSERT(config.getUnwrapRes().height <= 0xFF);
}
//----------------------------------------------------------------------------
const cv::Mat &ImageInputHorizon::processSnapshot(const cv::Mat &snapshot)
{
    // Read indices of segments
    const cv::Mat segmentedIndices = readSegmentIndices(snapshot);

    // Zero counts and sum of horizon pixels per column
    const int numColumns = segmentedIndices.size().width;
    m_ColumnHorizonPixelsSum.assign(numColumns, 0);
    m_ColumnHorizonPixelsCount.assign(numColumns, 0);
    BOB_ASSERT(m_ColumnHorizonPixelsSum.size() == (size_t)numColumns);
    BOB_ASSERT(m_ColumnHorizonPixelsCount.size() == (size_t)numColumns);

    // Loop through image columns
    for(auto p = segmentedIndices.begin<int32_t>(); p != segmentedIndices.end<int32_t>(); p++) {
        // If this is a horizon pixel
        if(*p == -1) {
            // Increment number of pixels per column
            m_ColumnHorizonPixelsCount[p.pos().x]++;

            // Add to total in horizon
            m_ColumnHorizonPixelsSum[p.pos().x] += p.pos().y;
        }
    }

    // Populate horizon vector with average horizon height
    std::transform(m_ColumnHorizonPixelsSum.cbegin(), m_ColumnHorizonPixelsSum.cend(), m_ColumnHorizonPixelsCount.cbegin(), m_HorizonVector.begin<uint8_t>(),
                   [](int sum, int count)
                   {
                       return (uint8_t)(sum / count);
                   });

    return m_HorizonVector;
}

std::unique_ptr<ImageInput> createImageInput(const Config &config)
{
    // Create image input
    if(config.shouldUseHorizonVector()) {
        LOGI << "Creating ImageInputHorizon";
        return std::make_unique<ImageInputHorizon>(config);
    }
    else if(config.shouldUseBinaryImage()) {
        LOGI << "Creating ImageInputBinary";
        return std::make_unique<ImageInputBinary>(config);
    }
    else {
        LOGI << "Creating ImageInputRaw";
        return std::make_unique<ImageInputRaw>(config);
    }
}
