// Standard C includes
#include <cstdio>

// Standard C++ includes
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <tuple>

// OpenCV
#include <opencv2/opencv.hpp>

// Third-party includes
#include "../third_party/path.h"
#include "../third_party/units.h"

// BoB robotics includes
#include "image_database.h"

namespace BoBRobotics {
using namespace units::length;
using namespace units::angle;
using namespace units::math;
using namespace units::literals;

class ImageDatabaseRecorder
{
public:
    ImageDatabaseRecorder(std::string databaseName, bool isRoute)
      : m_DatabasePath(databaseName), m_IsRoute(isRoute)
    {
        filesystem::create_directory(m_DatabasePath);

        // Write CSV header
        m_CSVStream.open((m_DatabasePath / "metadata.csv").str());
        m_CSVStream << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;
    }

    void saveImage(cv::Mat &frame, millimeter_t x, millimeter_t y, millimeter_t z, degree_t heading)
    {
        // Get image file name
        std::string filename = m_IsRoute ? getRouteDatabaseFilename(m_RouteCount++)
                                         : getImageDatabaseFilename((int) round(x), (int) round(y), (int) round(z));

        // Write current view to file
        cv::imwrite((filesystem::path(m_DatabasePath) / filename).str(), frame);

        // Write image file info to CSV file
        m_CSVStream << x.value() << ", " << y.value() << ", " << z.value() << ", "
                    << heading.value() << ", " << filename << std::endl;
    }

private:
    const filesystem::path m_DatabasePath;
    std::ofstream m_CSVStream;
    const bool m_IsRoute;
    unsigned int m_RouteCount = 0;
}; // ImageDatabaseRecorder
} // BoBRobotics