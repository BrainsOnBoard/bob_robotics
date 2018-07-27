// Standard C includes
#include <cstdio>

// Standard C++ includes
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/path.h"
#include "../third_party/units.h"

namespace BoBRobotics {
using namespace units::length;
using namespace units::angle;
using namespace units::math;
using namespace units::literals;

class ImageDatabaseRecorder
{
public:
    ImageDatabaseRecorder(std::string databaseName, bool isRoute)
      : m_DatabaseName(databaseName), m_IsRoute(isRoute)
    {
        filesystem::path databasePath = databaseName;
        filesystem::create_directory(databasePath);

        // Write CSV header
        m_CSVStream.open((databasePath / (databaseName + ".csv")).str());
        m_CSVStream << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;
    }

    void saveImage(cv::Mat &frame, millimeter_t x, millimeter_t y, millimeter_t z, degree_t heading)
    {
        // Get image file name
        char filename[255];
        if (m_IsRoute) {
            snprintf(filename, 255, "%s_%04d", m_DatabaseName.c_str(), m_RouteCount++);
        } else {
            snprintf(filename, 255, "%s_%05d_%05d_%05d.png",
                     m_DatabaseName.c_str(), (int) round(x), (int) round(y), (int) round(z));
        }
        auto imagePath = filesystem::path(m_DatabaseName) / filename;

        // Write current view to imagePath
        cv::imwrite(imagePath.str(), frame);

        // Write image file info to CSV file
        m_CSVStream << x.value() << ", " << y.value() << ", " << z.value() << ", "
                    << heading.value() << ", " << filename << std::endl;
    }

private:
    std::string m_DatabaseName;
    std::ofstream m_CSVStream;
    bool m_IsRoute;
    int m_RouteCount = 0;
}; // ImageDatabaseRecorder
} // BoBRobotics