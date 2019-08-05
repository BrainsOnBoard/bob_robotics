#pragma once

// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"
#include "video/panoramic.h"

// Forward declarations
class Config;

//----------------------------------------------------------------------------
// ImageInput
//----------------------------------------------------------------------------
class ImageInput
{
public:
    ImageInput(const Config &config);
    
    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual const cv::Mat &processSnapshot(const cv::Mat &snapshot) = 0;
    virtual cv::Size getOutputSize() const{ return m_InputSize; }
 
    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    const cv::Size &getInputSize() const{ return m_InputSize; }
    
private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    const cv::Size m_InputSize;
};

//----------------------------------------------------------------------------
// ImageInputRaw
//----------------------------------------------------------------------------
// Simply returns grayscale images from camera
class ImageInputRaw : public ImageInput
{
public:
    ImageInputRaw(const Config &config);
    
    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual const cv::Mat &processSnapshot(const cv::Mat &snapshot) override;

    
private:
    cv::Mat m_GreyscaleUnwrapped;
};

//----------------------------------------------------------------------------
// ImageInputBinary
//----------------------------------------------------------------------------
// Returns binary images, segmented into sky and ground using watershed algorithm
class ImageInputBinary : public ImageInput
{
public:
    ImageInputBinary(const Config &config);
    
    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual const cv::Mat &processSnapshot(const cv::Mat &snapshot) override;
    virtual cv::Size getOutputSize() const override { return cv::Size(getInputSize().width - 2, getInputSize().height - 2); }
    
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
    ImageInputHorizon(const Config &config);
    
    //----------------------------------------------------------------------------
    // ImageInput virtuals
    //----------------------------------------------------------------------------
    virtual const cv::Mat &processSnapshot(const cv::Mat &snapshot) override;
    
    virtual cv::Size getOutputSize() const override
    {
        return cv::Size(getInputSize().width - 2, 1);
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

std::unique_ptr<ImageInput> createImageInput(const Config &config);
