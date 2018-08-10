// Standard C includes
#include <cstdlib>

// OpenCV
#include <opencv2/opencv.hpp>

namespace BoBRobotics {
namespace Navigation {
class AbsDiff
{
public:
    AbsDiff(const int size)
      : m_Size(size)
    {}

    inline void calculateDifference(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &differenceImage)
    {
        cv::absdiff(image1, image2, differenceImage);
        m_Ptr = differenceImage.data;
    }

    inline uint8_t *begin()
    {
        return m_Ptr;
    }

    inline uint8_t *end()
    {
        return m_Ptr + m_Size;
    }

    static inline float mean(const float sum, const float n)
    {
        return sum / n;
    }

private:
    const int m_Size;
    uint8_t *m_Ptr;

};
} // Navigation
} // BoBRobotics
