#pragma once

#include "config.h"
#include "image_input.h"

// BoB robotics includes
#include "video/input.h"

// Standard C++ includes
#include <memory>

std::unique_ptr<BoBRobotics::Video::Input>
getPanoramicCamera(const Config &config);

std::unique_ptr<ImageInput>
createImageInput(const Config &config, const BoBRobotics::Video::Input &camera);
