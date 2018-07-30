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
        std::string filename;
        if (m_IsRoute) {
            char buf[255];
            snprintf(buf, 255, "%s_%04d.png", m_DatabaseName.c_str(), m_RouteCount++);
            filename = std::string(buf);
        } else {
            filename = m_DatabaseName + "_" + zeroPad(x) + "_" + zeroPad(y) + "_" + zeroPad(z) + ".png";
        }

        // Write current view to file
        cv::imwrite((filesystem::path(m_DatabaseName) / filename).str(), frame);

        // Write image file info to CSV file
        m_CSVStream << x.value() << ", " << y.value() << ", " << z.value() << ", "
                    << heading.value() << ", " << filename << std::endl;
    }

private:
    std::string m_DatabaseName;
    std::ofstream m_CSVStream;
    const bool m_IsRoute;
    int m_RouteCount = 0;

    static std::string zeroPad(millimeter_t value)
    {
        int ival = static_cast<int>(round(value));
        char num[12];
        snprintf(&num[1], 11, "%05d", abs(ival));
        num[0] = (ival < 0) ? '-' : '+';
        return std::string(num);
    }
}; // ImageDatabaseRecorder
} // BoBRobotics