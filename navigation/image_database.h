#pragma once

// BoB robotics includes
#include "../common/assert.h"
#include "../common/pose.h"

// Third-party includes
#include "../third_party/path.h"
#include "../third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C includes
#include <cstdio>

// Standard C++ includes
#include <algorithm>
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
            BOB_ASSERT(path.exists());
            return cv::imread(path.str());
        }

        cv::Mat loadGreyscale() const
        {
            BOB_ASSERT(path.exists());
            return cv::imread(path.str(), cv::IMREAD_GRAYSCALE);
        }
    };

    ImageDatabase(const char *databasePath)
      : ImageDatabase(filesystem::path(databasePath))
    {}

    ImageDatabase(const std::string &databasePath)
      : ImageDatabase(filesystem::path(databasePath))
    {}

    ImageDatabase(const filesystem::path &databasePath)
      : m_Path(databasePath)
    {
        const auto metadataPath = m_Path / MetadataFilename;

        // If we don't have metadata, it's an empty database
        if (!metadataPath.exists()) {
            // Make sure we have a directory to save into
            filesystem::create_directory(m_Path);
            return;
        }

        // Otherwise parse metadata file
        cv::FileStorage fs(metadataPath.str(), cv::FileStorage::READ);
        cv::FileNode entries = fs["entries"];
        const auto parse = [this](const cv::FileNode &node) {
            std::vector<double> pos;
            node["position_mm"] >> pos;
            BOB_ASSERT(pos.size() == 3);

            Entry entry;
            entry.position = { millimeter_t(pos[0]), millimeter_t(pos[1]), millimeter_t(pos[2]) };
            entry.heading = degree_t((double) node["heading_deg"]);
            entry.path = m_Path / ((std::string) node["filename"]);
            return entry;
        };
        std::transform(entries.begin(), entries.end(), std::back_inserter(m_Database), parse);
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

    void addImage(cv::Mat &frame, millimeter_t x, millimeter_t y, millimeter_t z, degree_t heading, bool isRoute)
    {
        // Get image file name
        std::string filename = isRoute ? getFilename(m_Database.size())
                                       : getFilename((int) round(x()), (int) round(y()), (int) round(z()));

        // Update database
        m_DirtyFlag = true;
        m_Database.push_back(Entry{ { x, y, z }, heading, filename });

        // Write current view to file
        const auto imagePath = m_Path / filename;
        BOB_ASSERT(!imagePath.exists()); // We don't want to overwrite data by default!
        cv::imwrite(imagePath.str(), frame);
    }

    void abortSave()
    {
        m_DirtyFlag = false;
    }

    void saveMetadata()
    {
        const std::string path = (m_Path / MetadataFilename).str();
        std::cout << "Writing metadata to " << path << "..." << std::endl;

        // Write image file info to YAML file
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "entries" << "[";
        for (auto &e : m_Database) {
            fs << "{:";
            fs << "position_mm"
               << "[:";
            for (auto p : e.position) {
                fs << p.value();
            }
            fs << "]";
            fs << "heading_deg" << e.heading.value();
            fs << "filename" << e.path.str();
            fs << "}";
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
    static constexpr const char *MetadataFilename = "metadata.yaml";

    static void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
        }));
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics