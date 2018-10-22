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
#include <ctime>

// Standard C++ includes
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::angle;
using namespace units::length;
using namespace units::literals;

//! A range of values in millimetres
struct Range
{
    const millimeter_t begin;
    const millimeter_t end;
    const millimeter_t separation;

    Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd, const millimeter_t separation)
      : begin(beginAndEnd.first)
      , end(beginAndEnd.second)
      , separation(separation)
    {
        if (begin == end) {
            BOB_ASSERT(separation == 0_mm);
        } else {
            BOB_ASSERT(begin < end);
            BOB_ASSERT(separation > 0_mm);
        }
    }

    Range(const millimeter_t value)
      : Range({value, value}, 0_mm)
    {}

    size_t size() const
    {
        return (separation == 0_mm) ? 1 : (1 + ((end - begin) / separation).to<size_t>());
    }
};

//! An interface for reading from and writing to folders of images
class ImageDatabase
{
public:
    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Vector3<millimeter_t> position;
        degree_t heading;
        filesystem::path path;
        Vector3<size_t> gridPosition; //! For grid-type databases, indicates the x,y,z grid position

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

    //! Base class for GridRecorder and RouteRecorder
    class Recorder {
    public:
        ~Recorder()
        {
            if (m_Recording) {
                save();
            }
        }

        //! Get an object for writing additional metadata to
        cv::FileStorage &getMetadataWriter() { return m_YAML; }

        //! Don't save new metadata when this class is destroyed
        void abortSave()
        {
            m_Recording = false;
        }

        //! Save new metadata
        void save()
        {
            // Write metadata to file
            {
                m_YAML << "}";

                const auto path = (m_ImageDatabase.m_Path / MetadataFilename).str();
                std::cout << "Writing metadata to " << path << "..." << std::endl;
                std::ofstream os(path);
                os << m_YAML.releaseAndGetString();
            }

            m_ImageDatabase.addNewEntries(m_NewEntries);
            m_Recording = false;
        }

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const { return m_NewEntries.size(); }

        //! Get the format in which images will be saved
        std::string getImageFormat() const { return m_ImageFormat; }

    private:
        ImageDatabase &m_ImageDatabase;
        const std::string m_ImageFormat;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        Recorder(ImageDatabase &imageDatabase, const bool isRoute, const std::string imageFormat)
          : m_ImageDatabase(imageDatabase)
          , m_ImageFormat(imageFormat)
          , m_Recording(true)
          , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
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
                      const Vector3<size_t> &gridPosition = { 0, 0, 0 })
        {
            BOB_ASSERT(m_Recording);
            m_ImageDatabase.writeImage(filename, image);
            m_NewEntries.emplace_back(Entry {
                position, heading, m_ImageDatabase.m_Path / filename, gridPosition
            });
        }
    };

    //! For recording a grid of images at a fixed heading
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

            // Save some extra, grid-specific metadata
            m_YAML << "grid" << "{"
                   << "beginAtMM" << "[:" << m_Begin[0]() << m_Begin[1]() << m_Begin[2]() << "]"
                   << "separationMM" << "[:" << m_Separation[0]() << m_Separation[1]() << m_Separation[2]() << "]"
                   << "size" << "[:" << (int) m_Size[0] << (int) m_Size[1] << (int) m_Size[2] << "]"
                   << "}";
        }

        //! Get the physical position represented by grid coordinates
        Vector3<millimeter_t> getPosition(const Vector3<size_t> &gridPosition)
        {
            BOB_ASSERT(gridPosition[0] < m_Size[0] && gridPosition[1] < m_Size[1] && gridPosition[2] < m_Size[2]);
            Vector3<millimeter_t> position;
            for (size_t i = 0; i < position.size(); i++) {
                position[i] = (m_Separation[i] * gridPosition[i]) + m_Begin[i];
            }
            return position;
        }

        //! Get a vector of all possible positions for this grid
        auto getPositions()
        {
            std::vector<Vector3<millimeter_t>> positions;
            positions.reserve(maximumSize());
            for (size_t x = 0; x < sizeX(); x++) {
                for (size_t y = 0; y < sizeY(); y++) {
                    for (size_t z = 0; z < sizeZ(); z++) {
                        positions.emplace_back(getPosition({ x, y, z }));
                    }
                }
            }
            return positions;
        }

        //! Save a new image into the database
        void record(const cv::Mat &image)
        {
            BOB_ASSERT(m_Current[2] < sizeZ());
            record(m_Current, image);

            if (++m_Current[0] == sizeX()) {
                m_Current[0] = 0;
                if (++m_Current[1] == sizeY()) {
                    m_Current[1] = 0;
                    m_Current[2]++;
                }
            }
        }

        //! Save a new image into the database at the specified coordinates
        void record(const Vector3<size_t> &gridPosition, const cv::Mat &image)
        {
            const auto position = getPosition(gridPosition);
            const std::string filename = ImageDatabase::getFilename(position, getImageFormat());
            addEntry(filename, image, position, m_Heading, { gridPosition[0], gridPosition[1], gridPosition[2] });
        }

        size_t maximumSize() const { return sizeX() * sizeY() * sizeZ(); }
        size_t sizeX() const { return m_Size[0]; }
        size_t sizeY() const { return m_Size[1]; }
        size_t sizeZ() const { return m_Size[2]; }

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const Vector3<size_t> m_Size;
        Vector3<size_t> m_Current;
    };

    //! For saving images recorded along a route
    class RouteRecorder : public Recorder {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, const std::string &imageFormat = "png")
          : Recorder(imageDatabase, true, imageFormat)
        {
            BOB_ASSERT(!imageDatabase.isGrid());
        }

        //! Save a new image taken at the specified pose
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
        const auto entriesPath = m_Path / EntriesFilename;

        // If we don't have any entries, it's an empty database
        if (!entriesPath.exists()) {
            // Make sure we have a directory to save into
            filesystem::create_directory(m_Path);
            return;
        }

        // Read metadata from YAML file
        const auto metadataPath = m_Path / MetadataFilename;
        const bool metadataPresent = metadataPath.exists();
        if (!metadataPresent) {
            std::cerr << "Warning, no " << MetadataFilename << " file found" << std::endl;
            m_IsRoute = true;
        } else {
            // Parse metadata file
            cv::FileStorage metadataFile(metadataPath.str(), cv::FileStorage::READ);

            // What type of database is it?
            std::string dbtype;
            metadataFile["metadata"]["type"] >> dbtype;
            if (dbtype == "route") {
                m_IsRoute = true;
            } else if (dbtype == "grid") {
                m_IsRoute = false;
            } else {
                throw std::runtime_error("Invalid database type \"" + dbtype + "\"");
            }
        }

        // Read in entries from CSV file
        std::ifstream entriesFile(metadataPath.str());
        std::string line, field;
        std::vector<std::string> fields;
        std::getline(entriesFile, line); // skip first line
        while (std::getline(entriesFile, line)) {
            std::stringstream lineStream(line);
            fields.clear();
            while (std::getline(lineStream, field, ',')) {
                // Trim whitespace from left (not C++ at its prettiest!)
                field.erase(field.begin(), std::find_if(field.begin(), field.end(), [](int ch) {
                    return !std::isspace(ch);
                }));
                fields.push_back(field);
            }
            if (fields.size() < 5) {
                break;
            }

            Vector3<size_t> gridPosition;
            if (fields.size() >= 8) {
                if (!metadataPresent) {
                    // Infer that it is a grid
                    m_IsRoute = false;
                }

                gridPosition[0] = static_cast<size_t>(std::stoul(fields[5]));
                gridPosition[1] = static_cast<size_t>(std::stoul(fields[6]));
                gridPosition[2] = static_cast<size_t>(std::stoul(fields[7]));
            } else {
                gridPosition = { 0, 0, 0 };
            }

            // Save details to vector
            Entry entry{
                { millimeter_t(std::stod(fields[0])),
                  millimeter_t(std::stod(fields[1])),
                  millimeter_t(std::stod(fields[2])) },
                degree_t(std::stod(fields[3])),
                m_Path / fields[4],
                gridPosition
            };
            m_Entries.push_back(entry);
        }
    }

    const filesystem::path &getPath() const { return m_Path; }

    const Entry &operator[](size_t i) const { return m_Entries[i]; }
    auto begin() const { return m_Entries.cbegin(); }
    auto end() const { return m_Entries.cend(); }
    size_t size() const { return m_Entries.size(); }
    bool empty() const { return m_Entries.empty(); }

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

    static std::string getFilename(const size_t routeIndex,
                                   const std::string &imageFormat = "png")
    {
        std::ostringstream ss;
        ss << "image_" << std::setw(5) << std::setfill('0') << routeIndex << "." << imageFormat;
        return ss.str();
    }

    static std::string getFilename(const Vector3<millimeter_t> &position,
                                   const std::string &imageFormat = "png")
    {
        // Convert to integers
        Vector3<int> iposition;
        std::transform(position.begin(), position.end(), iposition.begin(), [](auto mm) {
            return static_cast<int>(units::math::round(mm));
        });

        // Make filename
        std::ostringstream ss;
        ss << "image_" << std::setw(7) << std::setfill('0') << std::showpos << std::internal
           << std::setw(7) << iposition[0] << "_"
           << std::setw(7) << iposition[1] << "_"
           << std::setw(7) << iposition[2] << "." << imageFormat;
        return ss.str();
    }

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Entries;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    void writeImage(const std::string &filename, const cv::Mat &image)
    {
        const filesystem::path path = m_Path / filename;
        BOB_ASSERT(!path.exists()); // Don't overwrite data by default!
        cv::imwrite(path.str(), image);
    }

    void addNewEntries(std::vector<Entry> &newEntries)
    {
        if (newEntries.empty()) {
            std::cerr << "Warning: no new entries added, nothing will be written" << std::endl;
            return;
        }

        const std::string path = (m_Path / EntriesFilename).str();
        std::cout << "Writing entries to " << path << "..." << std::endl;

        // Move new entries into this object's vector
        m_Entries.reserve(m_Entries.size() + newEntries.size());
        for (auto &&e : newEntries) {
            m_Entries.emplace_back(std::move(e));
        }

        // Write image entries info to CSV file
        std::ofstream os(path);
        if (m_IsRoute) {
            os << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename" << std::endl;
            for (auto &e : m_Entries) {
                writeEntry(os, e);
                os << std::endl;
            }
        } else {
            os << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename, Grid X, Grid Y, Grid Z" << std::endl;
            for (auto &e : m_Entries) {
                writeEntry(os, e);
                os << ", " << e.gridPosition[0] << ", " << e.gridPosition[1] << ", " << e.gridPosition[2];
                os << std::endl;
            }
        }
    }

    void writeEntry(std::ofstream &os, const Entry &e)
    {
        os << e.position[0]() << ", " << e.position[1]() << ", "
           << e.position[2]() << ", " << e.heading() << ", " << e.path;
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics
