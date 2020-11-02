#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/pose.h"
#include "common/range.h"
#include "common/string.h"

// Third-party includes
#include "plog/Log.h"
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C includes
#include <ctime>

// Standard C++ includes
#include <array>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::ImageDatabase
//------------------------------------------------------------------------
//! An interface for reading from and writing to folders of images
class ImageDatabase
{
    using degree_t = units::angle::degree_t;
    using millimeter_t = units::length::millimeter_t;

public:
    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Vector3<millimeter_t> position = Vector3<millimeter_t>::nan();
        degree_t heading{ NAN };
        filesystem::path path;
        std::array<size_t, 3> gridPosition; //! For grid-type databases, indicates the x,y,z grid position
        std::unordered_map<std::string, std::string> extraFields;

        cv::Mat load(bool greyscale = true) const;
        bool hasExtraField(const std::string &name) const;
        const std::string &getExtraField(const std::string &name) const;
    };

    //! Base class for GridRecorder and RouteRecorder
    template<size_t NumExtraFields = 0>
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
        void abortSave() { m_Recording = false; }

        //! Save new metadata
        void save()
        {
            // Write metadata to file
            {
                m_YAML << "}";

                const auto path = (m_ImageDatabase.m_Path / MetadataFilename).str();
                LOG_INFO << "Writing metadata to " << path << "...";
                std::ofstream os(path);
                os << m_YAML.releaseAndGetString();
            }

            m_ImageDatabase.addNewEntries(m_NewEntries, m_ExtraFieldNames);
            m_Recording = false;
        }

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const { return m_NewEntries.size(); }

        //! Get the format in which images will be saved
        std::string getImageFormat() const { return m_ImageFormat; }

        void setImageFormat(const std::string &imageFormat)
        {
            m_ImageFormat = imageFormat;
        }

    private:
        const std::array<std::string, NumExtraFields> m_ExtraFieldNames;
        std::string m_ImageFormat = "png";
        ImageDatabase &m_ImageDatabase;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        template<class... Ts>
        Recorder(ImageDatabase &imageDatabase,
                 const bool isRoute,
                 Ts &&... extraFieldNames)
          : m_ExtraFieldNames({ std::forward<Ts>(extraFieldNames)... })
          , m_ImageDatabase(imageDatabase)
          , m_Recording(true)
          , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
        {
            // Set this property of the ImageDatabase
            imageDatabase.m_IsRoute = isRoute;

            // Get current date and time
            std::time_t now = std::time(nullptr);
            char timeStr[sizeof("0000-00-00 00:00:00")];
            BOB_ASSERT(0 != std::strftime(timeStr, sizeof(timeStr), "%F %T", std::localtime(&now)));

            // Write some metadata; users can add extra
            m_YAML << "metadata"
                   << "{"
                   << "time" << timeStr
                   << "project_git_commit" << BOB_PROJECT_GIT_COMMIT
                   << "bob_robotics_git_commit" << BOB_ROBOTICS_GIT_COMMIT
                   << "type" << (isRoute ? "route" : "grid");
        }

        template<class... Ts>
        void addEntry(const std::string &filename,
                      const cv::Mat &image,
                      const Vector3<millimeter_t> &position,
                      const degree_t heading,
                      const std::array<size_t, 3> &gridPosition,
                      Ts&&... extraFieldValues)
        {
            static_assert(sizeof...(extraFieldValues) == NumExtraFields,
                          "Must supply correct number of extra field values");
            BOB_ASSERT(m_Recording);

            m_ImageDatabase.writeImage(filename, image);

            Entry newEntry{
                position,
                heading,
                m_ImageDatabase.m_Path / filename,
                gridPosition,
                {}
            };
            m_NewEntries.emplace_back(std::move(newEntry));
            setExtraFields(std::forward<Ts>(extraFieldValues)...);
        }

        template<class... Ts>
        void setExtraFields(std::string value, Ts&&... otherValues)
        {
            const auto &key = *(m_ExtraFieldNames.cend() - 1 - sizeof...(otherValues));
            m_NewEntries.back().extraFields.emplace(key, std::move(value));
            setExtraFields(std::forward<Ts>(otherValues)...);
        }

        template<class T, class... Ts>
        void setExtraFields(T value, Ts&&... otherValues)
        {
            setExtraFields(std::to_string(value), std::forward<Ts>(otherValues)...);
        }

        // Stop condition
        void setExtraFields() const
        {}
    };

    //! For recording a grid of images at a fixed heading
    template<size_t NumExtraFields = 0>
    class GridRecorder
      : public Recorder<NumExtraFields> {
    public:
        template<class... Ts>
        GridRecorder(ImageDatabase &imageDatabase, const Range<millimeter_t> &xrange,
                     const Range<millimeter_t> &yrange, const Range<millimeter_t> &zrange = Range<millimeter_t>(0_mm),
                     degree_t heading = 0_deg, Ts &&... extraFieldNames)
          : Recorder<NumExtraFields>(imageDatabase, false,
                                     std::forward<Ts>(extraFieldNames)...)
          , m_Heading(heading)
          , m_Begin(*xrange.begin(), *yrange.begin(), *zrange.begin())
          , m_Separation(xrange.getStep(), yrange.getStep(), zrange.getStep())
          , m_Size({ xrange.size(), yrange.size(), zrange.size() })
          , m_Current({ 0, 0, 0 })
        {
            BOB_ASSERT(!imageDatabase.isRoute());

            // Save some extra, grid-specific metadata
            this->m_YAML << "grid"
                         << "{"
                         << "beginAtMM"
                         << "[:" << m_Begin[0]() << m_Begin[1]() << m_Begin[2]() << "]"
                         << "separationMM"
                         << "[:" << m_Separation[0]() << m_Separation[1]() << m_Separation[2]() << "]"
                         << "size"
                         << "[:" << (int) m_Size[0] << (int) m_Size[1] << (int) m_Size[2] << "]"
                         << "}";
        }

        //! Get the physical position represented by grid coordinates
        Vector3<millimeter_t> getPosition(const std::array<size_t, 3> &gridPosition) const
        {
            BOB_ASSERT(gridPosition[0] < m_Size[0]
                        && gridPosition[1] < m_Size[1]
                        && gridPosition[2] < m_Size[2]);
            Vector3<millimeter_t> position;
            for (size_t i = 0; i < position.size(); i++) {
                position[i] = (m_Separation[i] * gridPosition[i]) + m_Begin[i];
            }
            return position;
        }

        //! Get a vector of all possible positions for this grid
        std::vector<Vector3<millimeter_t>> getPositions()
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

        /**!
         * \brief Save a new image into the database with values for extra
         *        fields, if used
         */
        template<class... Ts>
        void record(const cv::Mat &image, Ts &&... extraFieldValues)
        {
            BOB_ASSERT(m_Current[2] < sizeZ());
            record(m_Current, image, std::forward<Ts>(extraFieldValues)...);

            if (++m_Current[0] == sizeX()) {
                m_Current[0] = 0;
                if (++m_Current[1] == sizeY()) {
                    m_Current[1] = 0;
                    m_Current[2]++;
                }
            }
        }

        /**!
         * \brief Save a new image into the database at the specified
         *        coordinates with values for extra fields, if used
         */
        template<class... Ts>
        void record(const std::array<size_t, 3> &gridPosition,
                    const cv::Mat &image, Ts&&... extraFieldValues)
        {
            const auto position = getPosition(gridPosition);
            const std::string filename = ImageDatabase::getFilename(position,
                this->getImageFormat());
            this->addEntry(filename, image, position, m_Heading,
                     { gridPosition[0], gridPosition[1], gridPosition[2] },
                     std::forward<Ts>(extraFieldValues)...);
        }

        size_t maximumSize() const
        {
            return sizeX() * sizeY() * sizeZ();
        }

        size_t sizeX() const { return m_Size[0]; }
        size_t sizeY() const { return m_Size[1]; }
        size_t sizeZ() const { return m_Size[2]; }

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const std::array<size_t, 3> m_Size;
        std::array<size_t, 3> m_Current;
    };

    //! For saving images recorded along a route
    template<size_t NumExtraFields = 0>
    class RouteRecorder
      : public Recorder<NumExtraFields> {
    public:
        template<class... Ts>
        RouteRecorder(ImageDatabase &imageDatabase, Ts&&... extraFieldNames)
          : Recorder<NumExtraFields>(imageDatabase, true,
                                     std::forward<Ts>(extraFieldNames)...)
        {
            BOB_ASSERT(!imageDatabase.isGrid());
        }

        /**!
         * \brief Save a new image taken at the specified pose with values for
         *        extra fields, if used
         */
        template<class... Ts>
        void record(const Vector3<millimeter_t> &position, degree_t heading,
                    const cv::Mat &image, Ts &&... extraFieldValues)
        {
            const auto filename = ImageDatabase::getFilename(this->size(),
                                                             this->getImageFormat());
            this->addEntry(filename, image, position, heading, { 0, 0, 0 },
                           std::forward<Ts>(extraFieldValues)...);
        }
    };

    ImageDatabase(const char *databasePath, bool overwrite = false);
    ImageDatabase(const std::string &databasePath, bool overwrite = false);
    ImageDatabase(const filesystem::path &databasePath, bool overwrite = false);

    //! Get the path of the directory corresponding to this ImageDatabase
    const filesystem::path &getPath() const;

    //! Get one Entry from the database
    const Entry &operator[](size_t i) const;

    //! Start iterator for the database entries
    std::vector<Entry>::const_iterator begin() const;

    //! End iterator for the database entries
    std::vector<Entry>::const_iterator end() const;

    //! Number of entries in this database
    size_t size() const;

    //! Check if there are any entries in this database
    bool empty() const;

    //! Check if the database is non-empty and a route-type database
    bool isRoute() const;

    //! Check if the database is non-empty and a grid-type database
    bool isGrid() const;

    //! Load all of the images in this database into memory and return
    std::vector<cv::Mat> loadImages(int type, const cv::Size &size = {}) const;

    //! Load all of the images in this database into the specified std::vector<>
    void loadImages(std::vector<cv::Mat> &images, int type,
                    const cv::Size &size = {}) const;


    // //! Load all of the images in this database into memory and return
    // template<class PixelType>
    // std::vector<cv::Mat>
    // loadImages(const cv::Size &size = {}, bool greyscale = true) const
    // {
    //     std::vector<cv::Mat> images;
    //     loadImages<PixelType>(images, size, greyscale);
    //     return images;
    // }

    // //! Load all of the images in this database into the specified std::vector<>
    // template<class PixelType>
    // void
    // loadImages(std::vector<cv::Mat> &images, const cv::Size &size = {},
    //            bool greyscale = true) const
    // {
    //     size_t oldSize = images.size();
    //     images.resize(oldSize + m_Entries.size());

    //     #pragma omp parallel for
    //     for (size_t i = 0; i < m_Entries.size(); i++) {
    //         cv::Mat &image = images[i + oldSize];
    //         image = m_Entries[i].load(greyscale);
    //         image.convertTo(image, CV_MAKE_TYPE(cv::DataType<PixelType>::depth,
    //                                             image.channels()));

    //         if (!size.empty()) {
    //             cv::resize(image, image, size);
    //         }
    //     }
    // }

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Start recording a grid of images
    template<class... Ts>
    auto getGridRecorder(const Range<millimeter_t> &xrange, const Range<millimeter_t> &yrange,
                         const Range<millimeter_t> &zrange = Range<millimeter_t>(0_mm),
                         degree_t heading = 0_deg, Ts &&... extraFieldNames)
    {
        return GridRecorder<sizeof...(extraFieldNames)>{
            *this, xrange, yrange, zrange, heading,
            std::forward<Ts>(extraFieldNames)...
        };
    }

    //! Start recording a route
    template<class... Ts>
    auto getRouteRecorder(Ts&&... extraFieldNames)
    {
        return RouteRecorder<sizeof...(extraFieldNames)>{
            *this, std::forward<Ts>(extraFieldNames)...
        };
    }

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination, const cv::Size &unwrapRes);

    //! Get a filename for a route-type database
    static std::string getFilename(const size_t routeIndex,
                                   const std::string &imageFormat = "png");

    //! Get a filename for a grid-type database
    static std::string getFilename(const Vector3<millimeter_t> &position,
                                   const std::string &imageFormat = "png");

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Entries;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    void loadMetadata();
    bool loadCSV();
    void readDirectoryEntries();
    void writeImage(const std::string &filename, const cv::Mat &image) const;

    template<size_t N>
    void addNewEntries(std::vector<Entry> &newEntries,
                       const std::array<std::string, N> &extraFieldNames)
    {
        if (newEntries.empty()) {
            LOG_WARNING << "No new entries added, nothing will be written";
            return;
        }

        const std::string path = (m_Path / EntriesFilename).str();
        LOG_INFO << "Writing entries to " << path << "...";

        // Move new entries into this object's vector
        m_Entries.reserve(m_Entries.size() + newEntries.size());
        for (auto &&e : newEntries) {
            m_Entries.emplace_back(std::move(e));
        }

        // Write image entries info to CSV file
        std::ofstream os(path);
        BOB_ASSERT(os.good());
        os << "X [mm], Y [mm], Z [mm], Heading [degrees], Filename";
        if (!m_IsRoute) {
            os << ", Grid X, Grid Y, Grid Z";
        }
        for (const auto &name : extraFieldNames) {
            os << ", " << name;
        }
        os << "\n";

        for (auto &e : m_Entries) {
            // These fields are always written...
            os << e.position[0]() << ", " << e.position[1]() << ", "
               << e.position[2]() << ", " << e.heading() << ", "
               << e.path.filename();

            // ...and these are only written if it's a grid database
            if (!m_IsRoute) {
                os << ", " << e.gridPosition[0] << ", " << e.gridPosition[1]
                   << ", " << e.gridPosition[2];
            }

            // Write any extra user-specified field values
            for (const auto &name : extraFieldNames) {
                os << ", " << e.extraFields[name];
            }

            os << "\n";
        }

        // Reload metadata, in case it's changed
        loadMetadata();
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics
