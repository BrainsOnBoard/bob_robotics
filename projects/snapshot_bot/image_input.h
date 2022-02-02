#pragma once

// BoB robotics includes
#include "imgproc/mask.h"
#include "imgproc/opencv_unwrap_360.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <memory>
#include <utility>
#include <vector>

// Forward declarations
class Config;

//----------------------------------------------------------------------------
// ImageInput
//----------------------------------------------------------------------------
class ImageInput
{
protected:
    using OpenCVUnwrap360 = BoBRobotics::ImgProc::OpenCVUnwrap360;
    using Mask = BoBRobotics::ImgProc::Mask;

public:
    ImageInput(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper);
    virtual ~ImageInput() = default;

    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual std::pair<cv::Mat, Mask> processSnapshot(const cv::Mat &snapshot) = 0;
    virtual cv::Size getOutputSize() const;
    virtual void writeMetadata(cv::FileStorage &fs) const;

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    cv::Size getUnwrapSize() const;

    //! Unwrap the image, crop and apply mask (if needed)
    std::pair<cv::Mat, Mask> preprocess(const cv::Mat &image) const;

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    mutable Mask m_Mask;
    mutable cv::Mat m_Unwrapped;
    const cv::Rect &m_CropRect;
    const cv::Size &m_InputSize;
    std::unique_ptr<OpenCVUnwrap360> m_Unwrapper;
    const bool m_UseODK;
};

//----------------------------------------------------------------------------
// ImageInputRaw
//----------------------------------------------------------------------------
// Simply returns grayscale images from camera
class ImageInputRaw : public ImageInput
{
public:
    ImageInputRaw(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper);

    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual std::pair<cv::Mat, Mask> processSnapshot(const cv::Mat &snapshot) override;

private:
    cv::Mat m_Greyscale;
};

//----------------------------------------------------------------------------
// ImageInputBinary
//----------------------------------------------------------------------------
// Returns binary images, segmented into sky and ground using watershed algorithm
class ImageInputBinary : public ImageInput
{
public:
    ImageInputBinary(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper);

    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual std::pair<cv::Mat, Mask> processSnapshot(const cv::Mat &snapshot) override;
    virtual cv::Size getOutputSize() const override { return cv::Size(getUnwrapSize().width - 2, getUnwrapSize().height - 2); }

protected:
    //----------------------------------------------------------------------------
    // Protected API
    //----------------------------------------------------------------------------
    // Reads unwrapped frame and performs segmentation
    cv::Mat readSegmentIndices(const cv::Mat &snapshot);

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    // Image containing watershed marker indices
    cv::Mat m_MarkerImage;

    // Image containing indices of segments
    cv::Mat m_SegmentIndices;

    // Resultant segmented image
    cv::Mat m_SegmentedImage;

    // Minimum segment in image
    int32_t m_MinIndex;

    // Maximum segment in image
    int32_t m_MaxIndex;
};

//----------------------------------------------------------------------------
// ImageInputHorizon
//----------------------------------------------------------------------------
//!
class ImageInputHorizon : public ImageInputBinary
{
public:
    ImageInputHorizon(const Config &config, const cv::Size &unwrapSize, std::unique_ptr<OpenCVUnwrap360> unwrapper);

    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual std::pair<cv::Mat, Mask> processSnapshot(const cv::Mat &snapshot) override;

    virtual cv::Size getOutputSize() const override
    {
        return cv::Size(getUnwrapSize().width - 2, 1);
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    // 'Image' containing average horizon height for each column
    cv::Mat m_HorizonVector;

    // Vectors holding the sum and count of horizon heights in each image column
    std::vector<int> m_ColumnHorizonPixelsSum;
    std::vector<int> m_ColumnHorizonPixelsCount;
};

std::unique_ptr<ImageInput> createImageInput(const Config &config,
                                             const cv::Size &unwrapSize,
                                             std::unique_ptr<BoBRobotics::ImgProc::OpenCVUnwrap360> unwrapper);
