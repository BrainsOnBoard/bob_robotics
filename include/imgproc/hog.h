#pragma once

// OpenCV
#include <opencv2/opencv.hpp>

class HOG {
public:
    HOG(const cv::Size &imageSize, const cv::Size &cellSize, int numOrientations)
      : m_HOG(createDescriptor(imageSize, cellSize, numOrientations))
      , m_HOGDescriptorSize(numOrientations * (imageSize.width / cellSize.width)
                            * (imageSize.height / cellSize.height))
    {
    }

    const int getNumHOGFeatures() const
    {
        return m_HOGDescriptorSize;
    }

private:
    const cv::HOGDescriptor m_HOG;
    const int m_HOGDescriptorSize;

    static cv::HOGDescriptor createDescriptor(const cv::Size &imageSize,
                                 const cv::Size &cellSize, int numOrientations)
    {
        // Configure HOG features - we want to normalise over the whole image (i.e. one block is the entire image)
        cv::HOGDescriptor hog;
        hog.winSize = imageSize;
        hog.blockSize = imageSize;
        hog.blockStride = imageSize;
        hog.cellSize = cellSize;
        hog.nbins = numOrientations;

        return hog;
    }
};
