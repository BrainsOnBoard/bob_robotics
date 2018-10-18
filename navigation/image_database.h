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
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
using namespace units::length;
using namespace units::literals;
using namespace units::math;

struct Range
{
    millimeter_t begin;
    millimeter_t end;
    millimeter_t separation;

    Range() = default;

    Range(const millimeter_t value)
    {
        begin = end = value;
        separation = 0_mm;
    }

    void check() const
    {
        if (begin == end) {
            BOB_ASSERT(separation == 0_mm);
        } else {
            BOB_ASSERT(begin < end);
            BOB_ASSERT(separation > 0_mm);
        }
    }

    size_t size() const
    {
        return (separation == 0_mm) ? 1 : ((end - begin) / separation).to<size_t>();
    }
};

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

    class GridRecorder {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange, const Range &yrange,
                     const Range &zrange = Range(0_mm), degree_t heading = 0_deg,
                     const std::string &imageFormat = "png")
          : m_ImageDatabase(imageDatabase)
          , m_Heading(heading)
          , m_Begin({ xrange.begin, yrange.begin, zrange.begin })
          , m_Separation({ xrange.separation, yrange.separation, zrange.separation })
          , m_Size({ xrange.size(), yrange.size(), zrange.size() })
          , m_ImageFormat(imageFormat)
        {
            BOB_ASSERT(!imageDatabase.isRoute());
            xrange.check();
            yrange.check();
            zrange.check();
        }

        ~GridRecorder()
        {
            if (m_Recording) {
                saveMetadata();
            }
        }

        Vector3<millimeter_t> getPosition(const Vector3<size_t> &indexes)
        {
            BOB_ASSERT(indexes[0] < m_Size[0] && indexes[1] < m_Size[1] && indexes[2] < m_Size[2]);
            Vector3<millimeter_t> position;
            for (size_t i = 0; i < position.size(); i++) {
                position[i] = (m_Separation[i] * indexes[i]) + m_Begin[i];
            }
            return position;
        }

        auto getPositions()
        {
            std::vector<Vector3<millimeter_t>> positions;
            const size_t s = size();
            positions.reserve(s);
            for (size_t i = 0; i < s; i++) {
                positions.emplace_back(getPosition(i));
            }
            return positions;
        }

        void record(const cv::Mat &image)
        {
            record(m_NewMetadata.size(), image);
        }

        template<typename IndexType>
        void record(const IndexType &indexes, const cv::Mat &image)
        {
            BOB_ASSERT(m_Recording);
            const auto position = getPosition(indexes);
            const std::string filename = ImageDatabase::getFilename(position, m_ImageFormat);

            m_ImageDatabase.writeImage(filename, image);
            m_NewMetadata.emplace_back(Entry { position, m_Heading, m_ImageDatabase.m_Path / filename});
        }

        void abortSave()
        {
            m_Recording = false;
        }

        void saveMetadata()
        {
            m_ImageDatabase.writeMetadata(m_NewMetadata, false);
            m_Recording = false;
        }

        size_t size() const { return sizeX() * sizeY() * sizeZ(); }
        size_t sizeX() const { return m_Size[0]; }
        size_t sizeY() const { return m_Size[1]; }
        size_t sizeZ() const { return m_Size[2]; }

    private:
        ImageDatabase &m_ImageDatabase;
        const degree_t m_Heading;
        std::vector<Entry> m_NewMetadata;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const Vector3<size_t> m_Size;
        const std::string m_ImageFormat;
        bool m_Recording = true;

        Vector3<millimeter_t> getPosition(size_t i)
        {
            const Vector3<size_t> indexes{ i % sizeX(), i / sizeX(), i / (sizeX() * sizeY()) };
            return getPosition(indexes);
        }
    };

    class RouteRecorder {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, const std::string &imageFormat = "png")
          : m_ImageDatabase(imageDatabase)
          , m_ImageFormat(imageFormat)
        {
            BOB_ASSERT(!imageDatabase.isGrid());
        }

        ~RouteRecorder()
        {
            if (m_Recording) {
                saveMetadata();
            }
        }

        void record(const Vector3<millimeter_t> &position, degree_t heading,
                    const cv::Mat &image)
        {
            BOB_ASSERT(m_Recording);

            const std::string filename = ImageDatabase::getFilename(m_NewMetadata.size(), m_ImageFormat);
            m_ImageDatabase.writeImage(filename, image);
            m_NewMetadata.emplace_back(Entry {
                position,
                heading,
                m_ImageDatabase.m_Path / filename,
            });
        }

        void abortSave()
        {
            m_Recording = false;
        }

        void saveMetadata()
        {
            m_ImageDatabase.writeMetadata(m_NewMetadata, true);
            m_Recording = false;
        }

        size_t size() const { return m_NewMetadata.size(); }

    private:
        ImageDatabase &m_ImageDatabase;
        const std::string m_ImageFormat;
        std::vector<Entry> m_NewMetadata;
        bool m_Recording = true;
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

            return Entry {
                { millimeter_t(pos[0]), millimeter_t(pos[1]), millimeter_t(pos[2]) },
                degree_t((double) node["heading_deg"]),
                m_Path / ((std::string) node["filename"])
            };
        };
        std::transform(entries.begin(), entries.end(), std::back_inserter(m_Metadata), parse);
    }

    const filesystem::path &getPath() const { return m_Path; }

    const Entry &operator[](size_t i) const { return m_Metadata[i]; }
    auto begin() const { return m_Metadata.cbegin(); }
    auto end() const { return m_Metadata.cend(); }
    size_t size() const { return m_Metadata.size(); }
    bool empty() const { return m_Metadata.empty(); }

    bool isRoute() const { return !empty() && m_IsRoute; }
    bool isGrid() const { return !empty() && !m_IsRoute; }

    GridRecorder getGridRecorder(const Range &xrange, const Range &yrange,
                                 const Range &zrange = Range(0_mm),
                                 degree_t heading = 0_deg,
                                 const std::string &imageFormat = "png")
    {
        return GridRecorder(*this, xrange, yrange, zrange, heading, imageFormat);
    }

    RouteRecorder getRouteRecorder(const std::string &imageFormat = "png")
    {
        return RouteRecorder(*this, imageFormat);
    }

    static std::string getFilename(const unsigned int routeIndex,
                                   const std::string &imageFormat = "png")
    {
        char buf[19];
        snprintf(buf, sizeof(buf), "image_%05d.", routeIndex);
        return std::string(buf) + imageFormat;
    }

    static std::string getFilename(const Vector3<millimeter_t> &position,
                                   const std::string &imageFormat = "png")
    {
        Vector3<int> iposition;
        std::transform(position.begin(), position.end(), iposition.begin(), [](auto mm) {
            return static_cast<int>(round(mm));
        });
        return getFilename(iposition, imageFormat);
    }

    static std::string getFilename(const Vector3<int> &position_mm,
                                   const std::string &imageFormat = "png")
    {
        const auto zeroPad = [](const auto value) {
            char num[12];
            snprintf(num, sizeof(num), "%+05d", value);
            return std::string(num);
        };
        return "image_" + zeroPad(position_mm[0]) + "_" + zeroPad(position_mm[1]) +
               "_" + zeroPad(position_mm[2]) + "." + imageFormat;
    }

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Metadata;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "metadata.yaml";

    void writeImage(const std::string &filename, const cv::Mat &image)
    {
        const filesystem::path path = m_Path / filename;
        BOB_ASSERT(!path.exists()); // Don't overwrite data by default!
        cv::imwrite(path.str(), image);
    }

    void writeMetadata(std::vector<Entry> &newMetadata, const bool isRoute)
    {
        m_IsRoute = isRoute;

        const std::string path = (m_Path / MetadataFilename).str();
        std::cout << "Writing metadata to " << path << "..." << std::endl;

        // Move new metadata into this object's vector
        m_Metadata.reserve(m_Metadata.size() + newMetadata.size());
        for (auto &&e : newMetadata) {
            m_Metadata.emplace_back(std::move(e));
        }

        // Write to YAML file
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "metadata"
           << "{:"
           << "is_route" << isRoute << "}";
        fs << "entries"
           << "[";
        for (auto &e : m_Metadata) {
            fs << "{:";
            fs << "position_mm"
               << "[:";
            for (auto p : e.position) {
                fs << p.value();
            }
            fs << "]";
            fs << "heading_deg" << e.heading.value();
            fs << "filename" << e.path.filename();
            fs << "}";
        }
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics