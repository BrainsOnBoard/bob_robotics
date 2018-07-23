#pragma once

// Standard C++ includes
#include <algorithm>
#include <functional>

// Standard C includes
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>

//------------------------------------------------------------------------
// BoBRobotics::GeNNUtils::VarSlide
//------------------------------------------------------------------------
namespace BoBRobotics {
namespace GeNNUtils {
template<typename T>
class VarSlider
{
public:
    VarSlider(T initial, T min, T max, std::function<void(T)> setValueFunction,
              const char *sliderText, const char *windowName)
    :   m_Min(min), m_Max(max), m_Percentage(toPercentage(initial)), m_SetValueFunction(setValueFunction)
    {
        cv::createTrackbar(sliderText, windowName, &m_Percentage, 100, &VarSlider::onTrackbar, this);
    }



private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    T fromPercentage(int percentage) const
    {
        return m_Min + ((m_Max - m_Min) * ((T)percentage / 100.0));
    }

    int toPercentage(T real) const
    {
        return (int)std::round(((real - m_Min) / (m_Max - m_Min)) * 100.0);
    }

    //------------------------------------------------------------------------
    // Static methods
    //------------------------------------------------------------------------
    static void onTrackbar(int value, void *context)
    {
        // Convert value from percentage to actual value
        VarSlider<T> *slider = reinterpret_cast<VarSlider<T>*>(context);
        slider->m_SetValueFunction(slider->fromPercentage(value));
    }

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    const T m_Min;
    const T m_Max;

    int m_Percentage;

    std::function<void(T)> m_SetValueFunction;
};
} // GeNNUtils
} // BoBRobotics
