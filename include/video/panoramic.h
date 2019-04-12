#pragma once

// BoB robotics includes
#include "input.h"

// OpenCV
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <memory>

namespace BoBRobotics {
namespace Video {

//! Try to find a panoramic camera on the current machine and return it
std::unique_ptr<Input>
getPanoramicCamera();

} // Video
} // BoBRobotics
