#pragma once

// BoB robotics includes
#include "pose.h"

// Third-party includes
#include "../third_party/path.h"
#include "../third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cassert>
#include <cstdio>

// Standard C++ includes
#include <fstream>
#include <string>
#include <vector>

namespace BoBRobotics {
using namespace units::length;
using namespace units::angle;

std::string
getRouteDatabaseFilename(const unsigned int routeIndex)
{
    char buf[22];
    snprintf(buf, sizeof(buf), "image_%05d.png", routeIndex);
    return std::string(buf);
}

std::string
getImageDatabaseFilename(const int x, const int y, const int z)
{
    const auto zeroPad = [](const auto value) {
        char num[12];
        snprintf(num, sizeof(num), "%+05d", value);
        return std::string(num);
    };
    return "image_" + zeroPad(x) + "_" + zeroPad(y) + "_" + zeroPad(z) + ".png";
}

class ImageDatabase
{
public:
    struct Entry {
        Vector3<millimeter_t> position;
        degree_t heading;
        std::string path;

        cv::Mat load() const
        {
            cv::Mat image;
            cv::imread(path);
            return image;
        }

        cv::Mat loadGreyscale() const
        {
            cv::Mat image;
            cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
            return image;
        }
    };

    ImageDatabase(std::string databaseName)
      : m_DatabasePath(databaseName)
    {
        // Read CSV header
        std::ifstream file((m_DatabasePath / "metadata.csv").str());
        std::string line;
        std::vector<std::string> fields;
        std::getline(file, line); // skip first line
        while (!file.eof()) {
            fields.clear();
            std::getline(file, line);
            size_t start = 0;
            for (size_t end = 0; end < line.size(); ) {
                if (line[end] == ',') {
                    fields.push_back(line.substr(start, end - start));
                    start = end + 1;
                    while (start < line.size() && line[start] == ' ') {
                        start++;
                    }
                    end = start;
                } else {
                    end++;
                }
            }
            fields.push_back(line.substr(start));
            if (fields.size() < 5) {
                break;
            }

            Entry entry {
                {
                    millimeter_t(std::stod(fields[0])),
                    millimeter_t(std::stod(fields[1])),
                    millimeter_t(std::stod(fields[2]))
                },
                degree_t(std::stod(fields[3])),
                (m_DatabasePath / fields[4]).str()};
            m_Database.push_back(entry);
        }
    }

    const Entry &operator[](size_t i) const { return m_Database[i]; }
    auto begin() const { return m_Database.cbegin(); }
    auto end() const { return m_Database.cend(); }
    size_t size() const { return m_Database.size(); }

private:
    const filesystem::path m_DatabasePath;
    std::vector<Entry> m_Database;
}; // ImageDatabase
} // BoB robotics