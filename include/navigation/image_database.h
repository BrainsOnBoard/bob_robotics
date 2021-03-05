#pragma once

// BoB robotics includes
#include "common/macros.h"
#include "common/path.h"
#include "common/pose.h"
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
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::Range
//------------------------------------------------------------------------
//! A range of values in millimetres
struct Range
{
    using millimeter_t = units::length::millimeter_t;
    millimeter_t begin, end, separation;

    constexpr Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd,
                    const millimeter_t separation)
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

    constexpr Range(const millimeter_t value)
      : Range({ value, value }, 0_mm)
    {}

    constexpr Range()
      : begin{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , end{ millimeter_t{ std::numeric_limits<double>::quiet_NaN() } }
      , separation{ 0_mm }
    {}

    size_t size() const;
};

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

    class FrameWriter {
    public:
        virtual std::string getCurrentFilenameRoot() const = 0;
        virtual void writeFrame(const cv::Mat &frame, Entry &entry) = 0;
    };

    class ImageFileWriter
      : public FrameWriter {
    public:
        ImageFileWriter(const ImageDatabase &) {}
        void setImageFormat(std::string);
        void writeFrame(const cv::Mat &frame, Entry &entry) override;

    private:
        std::string m_ImageFormat = "png";
    };

    class VideoFileWriter
      : public FrameWriter {
    public:
        VideoFileWriter(const ImageDatabase &);
        void writeFrame(const cv::Mat &frame, Entry &entry) override;
        const std::string &getVideoFileName() const;

    private:
        cv::VideoWriter m_Writer;
        const std::string m_FileName;
    };

    //! Base class for GridRecorder and RouteRecorder
    template<class FrameWriterType, size_t NumExtraFields = 0>
    class Recorder
      : public FrameWriterType
    {
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
                std::ofstream ofs(path);
                ofs.exceptions(std::ios::badbit | std::ios::failbit);
                ofs << m_YAML.releaseAndGetString();
            }

            m_ImageDatabase.addNewEntries(m_NewEntries, m_ExtraFieldNames);
            m_Recording = false;
        }

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const { return m_NewEntries.size(); }

    private:
        const std::array<std::string, NumExtraFields> m_ExtraFieldNames;
        ImageDatabase &m_ImageDatabase;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        template<class... Ts>
        Recorder(ImageDatabase &imageDatabase,
                 const bool isRoute,
                 Ts &&... extraFieldNames)
          : FrameWriterType{ imageDatabase }
          , m_ExtraFieldNames({ std::forward<Ts>(extraFieldNames)... })
          , m_ImageDatabase(imageDatabase)
          , m_Recording(true)
          , m_YAML(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY)
        {
            // Set this property of the ImageDatabase
            imageDatabase.m_IsRoute = isRoute;

            // Get current date and time
            const auto &creationTime = imageDatabase.getCreationTime();
            std::stringstream ss;
            ss << std::setfill('0')
               << std::setw(4) << (creationTime.tm_year + 1900) << "-"
               << std::setw(2) << creationTime.tm_mon << "-"
               << std::setw(2) << creationTime.tm_mday << " "
               << std::setw(2) << creationTime.tm_hour << ":"
               << std::setw(2) << creationTime.tm_min << ":"
               << std::setw(2) << creationTime.tm_sec;

            // Write some metadata; users can add extra
            m_YAML << "metadata"
                   << "{"
                   << "time" << ss.str()
                   << "project_git_commit" << BOB_PROJECT_GIT_COMMIT
                   << "bob_robotics_git_commit" << BOB_ROBOTICS_GIT_COMMIT
                   << "type" << (isRoute ? "route" : "grid");
        }

        template<class... Ts>
        void addEntry(const cv::Mat &image,
                      const Vector3<millimeter_t> &position,
                      const degree_t heading,
                      const std::array<size_t, 3> &gridPosition,
                      Ts&&... extraFieldValues)
        {
            static_assert(sizeof...(extraFieldValues) == NumExtraFields,
                          "Must supply correct number of extra field values");
            BOB_ASSERT(m_Recording);

            Entry newEntry{
                position,
                heading,
                "",
                gridPosition,
                {}
            };
            this->writeFrame(image, newEntry);
            m_NewEntries.emplace_back(std::move(newEntry));

            setExtraFields(std::forward<Ts>(extraFieldValues)...);
        }

        const ImageDatabase &getImageDatabase() const
        {
            return m_ImageDatabase;
        }

        template<class... Ts>
        void setExtraFields(std::string value, Ts&&... otherValues)
        {
            const auto &key = *(m_ExtraFieldNames.cend() - 1 - sizeof...(otherValues));
            m_NewEntries.back().extraFields.emplace(key, std::move(value));
            setExtraFields(std::forward<Ts>(otherValues)...);
        }

        template<class... Ts>
        void setExtraFields(const char *value, Ts&&... otherValues)
        {
            setExtraFields(std::string{ value }, std::forward<Ts>(otherValues)...);
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
      : public Recorder<ImageFileWriter, NumExtraFields> {
    public:
        template<class... Ts>
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange,
                     const Range &yrange, const Range &zrange = Range(0_mm),
                     degree_t heading = 0_deg, Ts &&... extraFieldNames)
          : Recorder<ImageFileWriter, NumExtraFields>(imageDatabase, false,
                                                      std::forward<Ts>(extraFieldNames)...)
          , m_Heading(heading)
          , m_Begin(xrange.begin, yrange.begin, zrange.begin)
          , m_Separation(xrange.separation, yrange.separation, zrange.separation)
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

        std::string getCurrentFilenameRoot() const override
        {
            // Convert to integers
            std::array<int, 3> iposition;
            const auto position = getPosition(m_Current);
            std::transform(position.begin(), position.end(), iposition.begin(), [](auto mm) {
                return static_cast<int>(units::math::round(mm));
            });

            // Make filename
            std::ostringstream ss;
            ss << "image_" << std::setw(7) << std::setfill('0') << std::showpos << std::internal
               << std::setw(7) << iposition[0] << "_"
               << std::setw(7) << iposition[1] << "_"
               << std::setw(7) << iposition[2];
            return (this->getImageDatabase().getPath() / ss.str()).str();
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
        std::vector<Vector3<millimeter_t>> getPositions() const
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
            this->addEntry(image, position, m_Heading,
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
    template<class FrameWriterType, size_t NumExtraFields = 0>
    class RouteRecorder
      : public Recorder<FrameWriterType, NumExtraFields> {
    public:
        template<class... Ts>
        RouteRecorder(ImageDatabase &imageDatabase, Ts&&... extraFieldNames)
          : Recorder<FrameWriterType, NumExtraFields>(imageDatabase, true,
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
            this->addEntry(image, position, heading, { 0, 0, 0 },
                           std::forward<Ts>(extraFieldValues)...);
        }

        std::string getCurrentFilenameRoot() const override
        {
            std::ostringstream ss;
            ss << "image" << std::setw(5) << std::setfill('0') << this->size() + 1;
            return (this->getImageDatabase().getPath() / ss.str()).str();
        }
    };

    ImageDatabase();
    ImageDatabase(const char *databasePath, bool overwrite = false);
    ImageDatabase(const std::string &databasePath, bool overwrite = false);
    ImageDatabase(filesystem::path databasePath, bool overwrite = false);
    ImageDatabase(const std::tm &creationTime);

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
    std::vector<cv::Mat> loadImages(const cv::Size &size = {}, bool greyscale = true) const;

    //! Load all of the images in this database into the specified std::vector<>
    void loadImages(std::vector<cv::Mat> &images, const cv::Size &size = {}, bool greyscale = true) const;

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Get the time at which this database was created
    const std::tm &getCreationTime() const;

    //! Start recording a grid of images
    template<class... Ts>
    auto getGridRecorder(const Range &xrange, const Range &yrange,
                         const Range &zrange = Range(0_mm),
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
        return RouteRecorder<ImageFileWriter, sizeof...(extraFieldNames)>{
            *this, std::forward<Ts>(extraFieldNames)...
        };
    }

    //! Start recording a route, saving images into video file
    template<class... Ts>
    auto getRouteVideoRecorder(const cv::Size &resolution, double fps,
                               Ts&&... extraFieldNames)
    {
        m_Resolution = resolution;
        m_FrameRate = fps;

        RouteRecorder<VideoFileWriter, sizeof...(extraFieldNames)> recorder{
            *this, std::forward<Ts>(extraFieldNames)...
        };
        recorder.getMetadataWriter() << "video_file" << recorder.getVideoFileName();

        return recorder;
    }

    double getFrameRate() const;

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    template<class Func>
    size_t forEachImage(const Func &func, bool greyscale = true) const
    {
        if (empty()) {
            return 0;
        }

        // If database consists of individual image files...
        if (m_VideoFileName.empty()) {
            size_t errors = 0;

            /*
             * You can't have uncaught exceptions eminating from OpenMP blocks,
             * so we just count the number of errors thrown instead.
             */
            #pragma omp parallel for shared(errors)
            for (size_t i = 0; i < m_Entries.size(); i++) {
                try {
                    func(i, m_Entries[i].load(greyscale));
                } catch (std::exception &e) {
                    LOGE << "Error occurred in forEachImage: " << e.what();

                    #pragma omp atomic
                    errors++;
                }
            }

            return errors;
        }

        // ...otherwise we have a video file
        cv::VideoCapture cap{ (m_Path / m_VideoFileName).str() };
        BOB_ASSERT(cap.isOpened());

        cv::Mat img;
        for (size_t i = 0; i < m_Entries.size(); i++) {
            try {
                BOB_ASSERT(cap.read(img));

                if (greyscale) {
                    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                }

                func(i, img);
            } catch (std::exception &e) {
                LOGE << "Error occurred in forEachImage: " << e.what();
                return m_Entries.size() - i;
            }
        }

        // Signal that all images were processed correctly
        return 0;
    }

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination,
                const cv::Size &unwrapRes) const;

    //! Return true if fn1 should be sorted before fn2
    static bool fileNameCompare(const std::string &fn1, const std::string &fn2);

private:
    filesystem::path m_Path;
    std::string m_VideoFileName;
    std::vector<Entry> m_Entries;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    std::tm m_CreationTime;
    double m_FrameRate = 0.0;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    ImageDatabase(const std::tm *creationTime, filesystem::path databasePath,
                  bool overwrite);

    void loadMetadata();
    bool loadCSV();
    bool readDirectoryEntries();
    void writeImage(const std::string &filename, const cv::Mat &image) const;

    template<size_t N>
    void addNewEntries(std::vector<Entry> &newEntries,
                       const std::array<std::string, N> &extraFieldNames)
    {
        if (newEntries.empty()) {
            LOG_WARNING << "No new entries added, nothing will be written";
            return;
        }

        // Reload metadata, in case it's changed
        loadMetadata();

        const std::string path = (m_Path / EntriesFilename).str();
        LOG_INFO << "Writing entries to " << path << "...";

        // Move new entries into this object's vector
        m_Entries.reserve(m_Entries.size() + newEntries.size());
        for (auto &&e : newEntries) {
            m_Entries.emplace_back(std::move(e));
        }

        // Write image entries info to CSV file
        std::ofstream os;
        os.exceptions(std::ios::badbit | std::ios::failbit);
        os.open(path);
        os << "X [mm], Y [mm], Z [mm], Heading [degrees]";
        if (m_VideoFileName.empty()) {
            os << ", Filename";
        }
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
               << e.position[2]() << ", " << e.heading();

            // ...this is only written if we're not saving as a video
            if (m_VideoFileName.empty()) {
                os << ", " << e.path.filename();
            }

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
    }
}; // ImageDatabase
} // Navigation
} // BoB robotics
