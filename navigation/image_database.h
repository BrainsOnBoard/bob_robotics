#pragma once

// BoB robotics includes
#include "../common/pose.h"

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
#include <iostream>
#include <string>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::length;
using namespace units::angle;

class ImageDatabase
{
public:
    struct Entry
    {
        Vector3<millimeter_t> position;
        degree_t heading;
        filesystem::path path;

        cv::Mat load() const
        {
            assert(path.exists());
            return cv::imread(path.str());
        }

        cv::Mat loadGreyscale() const
        {
            assert(path.exists());
            return cv::imread(path.str(), cv::IMREAD_GRAYSCALE);
        }
    };

    ImageDatabase(const char *databasePath)
      : m_Path(databasePath)
    {}

    ImageDatabase(const std::string &databasePath)
      : m_Path(databasePath)
    {}

    ImageDatabase(const filesystem::path &databasePath)
      : m_Path(databasePath)
    {
        const auto metadataPath = m_Path / "metadata.csv";

        // If we don't have metadata, it's an empty database
        if (!metadataPath.exists()) {
            // Make sure we have a directory to save into
            filesystem::create_directory(m_Path);
            return;
        }

        // Otherwise parse metadata file
        std::ifstream file(metadataPath.str());
        std::string line;
        std::vector<std::string> fields;
        std::getline(file, line); // skip first line
        while (!file.eof()) {
            // Values are separated by commas
            fields.clear();
            std::getline(file, line);
            size_t start = 0;
            for (size_t end = 0; end < line.size(); ) {
                if (line[end] == ',') {
                    fields.push_back(line.substr(start, end - start));
                    start = end + 1;

                    // Skip whitespace
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

            // Save details to vector
            Entry entry {
                {
                    millimeter_t(std::stod(fields[0])),
                    millimeter_t(std::stod(fields[1])),
                    millimeter_t(std::stod(fields[2]))
                },
                degree_t(std::stod(fields[3])),
                (m_Path / fields[4]).str()};
            m_Database.push_back(entry);
        }
    }

    ~ImageDatabase()
    {
        // If we need to write metadata and haven't yet done so, do it now
        if (m_DirtyFlag) {
            saveMetadata();
        }
    }

    const filesystem::path &getPath() const { return m_Path; }

    const Entry &operator[](size_t i) const { return m_Database[i]; }
    auto begin() const { return m_Database.cbegin(); }
    auto end() const { return m_Database.cend(); }
    size_t size() const { return m_Database.size(); }

    void saveImage(cv::Mat &frame, millimeter_t x, millimeter_t y, millimeter_t z, degree_t heading, bool isRoute)
    {
        // Get image file name
        std::string filename = isRoute ? getFilename(m_Database.size())
                                       : getFilename((int) round(x()), (int) round(y()), (int) round(z()));

        // Update database
        m_DirtyFlag = true;
        m_Database.push_back(Entry{{x, y, z}, heading, filename});

        // Write current view to file
        cv::imwrite((filesystem::path(m_Path) / filename).str(), frame);
    }

    void saveMetadata()
    {
        const std::string path = (m_Path / "metadata.csv").str();
        std::cout << "Writing metadata to " << path << "..." << std::endl;

        // Open file and write header
        std::ofstream file(path);
        file << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;

        // Write image file info to CSV file
        for (auto &e : m_Database) {
            file << e.position[0]() << ", " << e.position[1]() << ", "
                 << e.position[2]() << ", " << e.heading() << ", " << e.path << std::endl;
        }

        // Metadata no longer needs to be written
        m_DirtyFlag = false;
    }

    static std::string getFilename(const unsigned int routeIndex)
    {
        char buf[22];
        snprintf(buf, sizeof(buf), "image_%05d.png", routeIndex);
        return std::string(buf);
    }

    static std::string getFilename(const int x, const int y, const int z)
    {
        const auto zeroPad = [](const auto value) {
            char num[12];
            snprintf(num, sizeof(num), "%+05d", value);
            return std::string(num);
        };
        return "image_" + zeroPad(x) + "_" + zeroPad(y) + "_" + zeroPad(z) + ".png";
    }

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Database;
    bool m_DirtyFlag = false;
}; // ImageDatabase
} // Navigation
} // BoB robotics