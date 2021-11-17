// Snapshot bot includes
#include "config.h"
#include "image_input.h"

// BoB robotics includes
#include "common/macros.h"

// Third-party includes
#include "plog/Log.h"

// using namespace BoBRobotics;
using namespace BoBRobotics::ImgProc;

namespace {
// Bounds used for extracting masks from ODK2 images
const cv::Scalar odk2MaskLowerBound(1, 1, 1);
const cv::Scalar odk2MaskUpperBound(255, 255, 255);
}

//----------------------------------------------------------------------------
// ImageInput
//----------------------------------------------------------------------------
ImageInput::ImageInput(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper)
  : m_Unwrapped(unwrapSize, CV_8UC3)
  , m_CropRect(config.getCroppedRect())
  , m_InputSize(unwrapSize)
  , m_Unwrapper(std::move(unwrapper))
  , m_UseODK(config.shouldUseODK2())
{
    // If a static mask image is specified, set it as the mask
    if(!config.getMaskImageFilename().empty()) {
        if (m_UseODK) {
            LOGW << "Cannot use a mask file if ODK is in use! It will be ignored";
        } else {
            m_Mask.set(config.getMaskImageFilename());
        }
    }
}
//----------------------------------------------------------------------------
cv::Size ImageInput::getUnwrapSize() const
{
    return m_Unwrapped.size();
}
//----------------------------------------------------------------------------
void ImageInput::writeMetadata(cv::FileStorage &fs) const
{
    fs << "imageInput" << "{";

    if (m_Unwrapper) {
        fs << "unwrapper" << *m_Unwrapper;
    }

    fs << "}";
}
//----------------------------------------------------------------------------
cv::Size ImageInput::getOutputSize() const
{
    return m_CropRect.size();
}
//----------------------------------------------------------------------------
std::pair<cv::Mat, Mask> ImageInput::preprocess(const cv::Mat &image) const
{
    if (m_Unwrapper) {
        // Unwrap panorama
        m_Unwrapper->unwrap(image, m_Unwrapped);
    } else {
        m_Unwrapped = image;
    }

    // Crop the image to the bounds specified in the config file
    cv::Mat cropped{ m_Unwrapped, m_CropRect };

    // Images from the ODK2 need a dynamic masking process
    if (m_UseODK) {
        m_Mask.set(cropped, odk2MaskLowerBound, odk2MaskUpperBound);
    }

    /*
     * Return cropped image and mask. Note that this operation will not perform
     * any heap allocations as cv::Mats use CoW internally.
     */
    return { cropped, m_Mask };
}
//----------------------------------------------------------------------------
// ImageInputRaw
//----------------------------------------------------------------------------
ImageInputRaw::ImageInputRaw(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper)
  : ImageInput(config, unwrapSize, std::move(unwrapper))
  , m_Greyscale(getUnwrapSize(), CV_8UC1)
{
}
//----------------------------------------------------------------------------
std::pair<cv::Mat, Mask> ImageInputRaw::processSnapshot(const cv::Mat &snapshot)
{
    // Convert to greyscale
    cv::cvtColor(snapshot, m_Greyscale, cv::COLOR_BGR2GRAY);

    // Return cropped and unwrapped image
    return preprocess(m_Greyscale);
}

//----------------------------------------------------------------------------
// ImageInputBinary
//----------------------------------------------------------------------------
ImageInputBinary::ImageInputBinary(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper)
  : ImageInput(config, unwrapSize, std::move(unwrapper))
  , m_SegmentIndices(config.getCroppedRect().size(), CV_32SC1)
  , m_SegmentedImage(config.getCroppedRect().height - 2, config.getCroppedRect().width - 2, CV_8UC1)
{
    // Read marker image
    // **NOTE** will read 8-bit per channel grayscale
    m_MarkerImage = cv::imread(config.getWatershedMarkerImageFilename(), cv::IMREAD_GRAYSCALE);
    BOB_ASSERT(m_MarkerImage.size() == config.getCroppedRect().size());

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
std::pair<cv::Mat, Mask> ImageInputBinary::processSnapshot(const cv::Mat &snapshot)
{
    // Read indices of segments
    auto imageAndMask = preprocess(snapshot);
    const cv::Mat segmentedIndices = readSegmentIndices(imageAndMask.first);

    // Convert to greyscale image
    // **NOTE** segmentedIndices should contain -1, 1 and 2 - this rescales to black, white and grey
    segmentedIndices.convertTo(m_SegmentedImage, CV_8UC1, 85.0, 85.0);
    BOB_ASSERT(m_SegmentedImage.size() == getOutputSize());
    return { m_SegmentedImage, imageAndMask.second };
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
    return cv::Mat(m_SegmentIndices, cv::Rect(1, 1, getUnwrapSize().width - 2, getUnwrapSize().height - 2));
}

//----------------------------------------------------------------------------
// ImageInputHorizon
//----------------------------------------------------------------------------
ImageInputHorizon::ImageInputHorizon(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper)
  : ImageInputBinary(config, unwrapSize, std::move(unwrapper))
  , m_HorizonVector(1, config.getCroppedRect().width - 2, CV_8UC1)
  , m_ColumnHorizonPixelsSum(config.getCroppedRect().width - 2)
  , m_ColumnHorizonPixelsCount(config.getCroppedRect().width - 2)
{
    // Check image will be representable as 8-bit value
    BOB_ASSERT(config.getCroppedRect().height <= 0xFF);
}
//----------------------------------------------------------------------------
std::pair<cv::Mat, Mask> ImageInputHorizon::processSnapshot(const cv::Mat &snapshot)
{
    // Read indices of segments
    auto imageAndMask = preprocess(snapshot);
    const cv::Mat segmentedIndices = readSegmentIndices(imageAndMask.first);

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

    return { m_HorizonVector, imageAndMask.second };
}

std::unique_ptr<ImageInput> createImageInput(const Config& config,
                                             const cv::Size &unwrapSize,
                                             std::unique_ptr<OpenCVUnwrap360> unwrapper)
{
    // Create image input
    if(config.shouldUseHorizonVector()) {
        LOGI << "Creating ImageInputHorizon";
        return std::make_unique<ImageInputHorizon>(config, unwrapSize, std::move(unwrapper));
    }
    else if(config.shouldUseBinaryImage()) {
        LOGI << "Creating ImageInputBinary";
        return std::make_unique<ImageInputBinary>(config, unwrapSize, std::move(unwrapper));
    }
    else {
        LOGI << "Creating ImageInputRaw";
        return std::make_unique<ImageInputRaw>(config, unwrapSize, std::move(unwrapper));
    }
}
