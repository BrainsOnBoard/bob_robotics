#pragma once

// Standard C includes
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

//------------------------------------------------------------------------
// BoBRobotics::GeNNUtils::SpikeRenderer
//------------------------------------------------------------------------
namespace BoBRobotics {
namespace GeNNUtils {
class SpikeRenderer
{
public:
    SpikeRenderer(unsigned int *&spkCnt, unsigned int *&spk, unsigned int width, unsigned int height, double tau)
    :   m_SpikeImage(height, width, CV_8UC1), m_SpikeImageFloat(height, width, CV_32FC1, 0.0f), m_Decay(std::exp(-1.0f / tau)),
        m_SpkCnt(spkCnt), m_Spk(spk)
    {

    }

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    void update()
    {
        // Apply decay
        m_SpikeImageFloat *= m_Decay;
        float *spikeImageFloatRaw = reinterpret_cast<float*>(m_SpikeImageFloat.data);
        for(unsigned int i = 0; i < m_SpkCnt[0]; i++) {
            spikeImageFloatRaw[m_Spk[i]] += 1.0f;
        }

        m_SpikeImageFloat.convertTo(m_SpikeImage, CV_8UC1, 255.0);
    }


    cv::Mat &getSpikeImage(){ return m_SpikeImage; }
    const cv::Mat &getSpikeImage() const{ return m_SpikeImage; }

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    cv::Mat m_SpikeImage;
    cv::Mat m_SpikeImageFloat;

    const double m_Decay;

    unsigned int *&m_SpkCnt;
    unsigned int *&m_Spk;
};
} // GeNNUtils
} // BoBRobotics
