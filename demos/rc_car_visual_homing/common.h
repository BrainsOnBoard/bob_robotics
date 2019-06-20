#pragma once

// BoB robotics includes
#include "common/gps.h"
#include "common/logging.h"
#include "common/macros.h"

// Standard C++ includes
#include "common/threadable.h"

// Standard C includes
#include <cstring>

// Standard C++ includes
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

constexpr const char *imageFolderPrefix = "training_images";

using namespace BoBRobotics;

std::string
getImageFilename(int num)
{
    std::stringstream ss;
    ss << "image" << std::setw(3) << std::setfill('0') << num << ".png";
    return ss.str();
}

void
writeGPSData(std::ofstream &logWriter, GPS::GPSData &data)
{
    logWriter << data.coordinate.lat.value() << ", "
              << data.coordinate.lon.value() << ", "
              << data.altitude.value() << ", "
              << data.velocity.value() << ", "
              << data.numberOfSatellites << ", "
              << data.time.str()
              << "\n";
}

