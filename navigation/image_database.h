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
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
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
    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Vector3<millimeter_t> position;
        degree_t heading;
        filesystem::path path;
        Vector3<int> indexes{ -1, -1, -1 }; //! For grid-type databases, indicates the x,y,z grid position

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

    class Recorder {
    public:
        ~Recorder()
        {
            if (m_Recording) {
                save();
            }
        }

        cv::FileStorage &getMetadataWriter() { return m_YAML; }

        void abortSave()
        {
            m_Recording = false;
        }

        void save()
        {
            // End writing metadata
            m_YAML << "}";

            m_ImageDatabase.addNewEntries(m_NewEntries, m_YAML);
            m_Recording = false;
        }

        size_t size() const { return m_NewEntries.size(); }

        std::string getImageFormat() { return m_ImageFormat; }

    private:
        ImageDatabase &m_ImageDatabase;
        const std::string m_ImageFormat;
        cv::FileStorage m_YAML;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        Recorder(ImageDatabase &imageDatabase, const bool isRoute, const std::string imageFormat)
          : m_ImageDatabase(imageDatabase)
          , m_ImageFormat(imageFormat)
          , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
          , m_Recording(true)
        {
            // Set this property of the ImageDatabase
            imageDatabase.m_IsRoute = isRoute;

            // Get current date and time
            std::time_t now = std::time(nullptr);
            char timeStr[sizeof("0000-00-00 00:00:00")];
            BOB_ASSERT(0 != std::strftime(timeStr, sizeof(timeStr), "%F %T",
                                          std::localtime(&now)));

            // Write some metadata; users can add extra
            m_YAML << "metadata"
                   << "{"
                   << "time" << timeStr
                   << "type" << (isRoute ? "route" : "grid");
        }

        void addEntry(const std::string &filename, const cv::Mat &image,
                      const Vector3<millimeter_t> &position, const degree_t heading,
                      const Vector3<int> &indexes = { -1, -1, -1 })
        {
            BOB_ASSERT(m_Recording);
            m_ImageDatabase.writeImage(filename, image);
            m_NewEntries.emplace_back(Entry {
                position, heading, m_ImageDatabase.m_Path / filename, indexes
            });
        }
    };

    class GridRecorder : public Recorder {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange, const Range &yrange,
                     const Range &zrange = Range(0_mm), degree_t heading = 0_deg,
                     const std::string &imageFormat = "png")
          : Recorder(imageDatabase, false, imageFormat)
          , m_Heading(heading)
          , m_Begin({ xrange.begin, yrange.begin, zrange.begin })
          , m_Separation({ xrange.separation, yrange.separation, zrange.separation })
          , m_Size({ xrange.size(), yrange.size(), zrange.size() })
          , m_Current({ 0, 0, 0 })
        {
            BOB_ASSERT(!imageDatabase.isRoute());
            xrange.check();
            yrange.check();
            zrange.check();
        }

        Vector3<millimeter_t> getPosition(const Vector3<int> &indexes)
        {
            BOB_ASSERT(indexes[0] < (int) m_Size[0] && indexes[1] < (int) m_Size[1] && indexes[2] < (int) m_Size[2]);
            Vector3<millimeter_t> position;
            for (size_t i = 0; i < position.size(); i++) {
                position[i] = (m_Separation[i] * indexes[i]) + m_Begin[i];
            }
            return position;
        }

        auto getPositions()
        {
            std::vector<Vector3<millimeter_t>> positions;
            positions.reserve(maximumSize());
            for (int x = 0; x < (int) sizeX(); x++) {
                for (int y = 0; y < (int) sizeY(); y++) {
                    for (int z = 0; z < (int) sizeZ(); z++) {
                        positions.emplace_back(getPosition({ x, y, z }));
                    }
                }
            }
            return positions;
        }

        void record(const cv::Mat &image)
        {
            BOB_ASSERT((size_t) m_Current[2] < sizeZ());
            record(m_Current, image);

            if ((size_t) m_Current[0] < sizeX()) {
                m_Current[0]++;
            } else {
                m_Current[0] = 0;
                if ((size_t) m_Current[1] < sizeY()) {
                    m_Current[1]++;
                } else {
                    m_Current[1] = 0;
                    m_Current[2]++;
                }
            }
        }

        void record(const Vector3<int> &indexes, const cv::Mat &image)
        {
            const auto position = getPosition(indexes);
            const std::string filename = ImageDatabase::getFilename(position, getImageFormat());
            addEntry(filename, image, position, m_Heading, { indexes[0], indexes[1], indexes[2] });
        }

        size_t maximumSize() const { return sizeX() * sizeY() * sizeZ(); }
        size_t sizeX() const { return m_Size[0]; }
        size_t sizeY() const { return m_Size[1]; }
        size_t sizeZ() const { return m_Size[2]; }

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const Vector3<size_t> m_Size;
        Vector3<int> m_Current;
    };

    class RouteRecorder : public Recorder {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, const std::string &imageFormat = "png")
          : Recorder(imageDatabase, true, imageFormat)
        {
            BOB_ASSERT(!imageDatabase.isGrid());
        }

        void record(const Vector3<millimeter_t> &position, degree_t heading,
                    const cv::Mat &image)
        {
            const std::string filename = ImageDatabase::getFilename(size(), getImageFormat());
            addEntry(filename, image, position, heading);
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
        cv::FileStorage yaml(metadataPath.str(), cv::FileStorage::READ);

        // What type of database is it?
        std::string dbtype;
        yaml["metadata"]["type"] >> dbtype;
        if (dbtype == "route") {
            m_IsRoute = true;
        } else if (dbtype == "grid") {
            m_IsRoute = false;
        } else {
            throw std::runtime_error("Invalid database type \"" + dbtype + "\"");
        }

        // Read in database entries
        cv::FileNode entries = yaml["entries"];
        const auto parse = [this](const cv::FileNode &node) {
            std::vector<double> pos;
            node["positionMM"] >> pos;
            BOB_ASSERT(pos.size() == 3);

            std::vector<int> indexes;
            if (m_IsRoute) {
                indexes = { -1, -1, -1 };
            } else {
                node["grid"] >> indexes;
            }

            return Entry {
                { millimeter_t(pos[0]), millimeter_t(pos[1]), millimeter_t(pos[2]) },
                degree_t((double) node["headingDegrees"]),
                m_Path / ((std::string) node["filename"]),
                { indexes[0], indexes[1], indexes[2] }
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
        char buf[sizeof("image_00000.") + 1];
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

    static std::string getFilename(const Vector3<int> &positionMM,
                                   const std::string &imageFormat = "png")
    {
        const auto zeroPad = [](const auto value) {
            char num[8];
            snprintf(num, sizeof(num), "%+07d", value);
            return std::string(num);
        };
        return "image_" + zeroPad(positionMM[0]) + "_" + zeroPad(positionMM[1]) +
               "_" + zeroPad(positionMM[2]) + "." + imageFormat;
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

    void addNewEntries(std::vector<Entry> &newEntries, cv::FileStorage &yaml)
    {
        if (newEntries.empty()) {
            std::cerr << "Warning: no new entries added, nothing will be written" << std::endl;
            return;
        }

        const std::string path = (m_Path / MetadataFilename).str();
        std::cout << "Writing metadata to " << path << "..." << std::endl;

        // Move new metadata into this object's vector
        m_Metadata.reserve(m_Metadata.size() + newEntries.size());
        for (auto &&e : newEntries) {
            m_Metadata.emplace_back(std::move(e));
        }

        // Add entries to YAML object
        yaml << "entries"
             << "[";
        for (auto &e : m_Metadata) {
            yaml << "{:";

            if (!m_IsRoute) {
                yaml << "grid"
                     << "[:";
                for (auto i : e.indexes) {
                    yaml << i;
                }
                yaml << "]";
            }

            yaml << "positionMM"
                 << "[:";
            for (auto p : e.position) {
                yaml << p.value();
            }
            yaml << "]";
            yaml << "headingDegrees" << e.heading.value();
            yaml << "filename" << e.path.filename();
            yaml << "}";
        }
        yaml << "]";

        // Write to file
        std::ofstream os(path);
        os << yaml.releaseAndGetString();
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics