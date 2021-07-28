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

// TBB
#include <tbb/parallel_for.h>

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
    using hertz_t = units::frequency::hertz_t;

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
        ImageFileWriter(const ImageDatabase &, std::string imageFormat);
        void writeFrame(const cv::Mat &frame, Entry &entry) override;

    private:
        const std::string m_ImageFormat;
    };

    class VideoFileWriter
      : public FrameWriter {
    public:
        VideoFileWriter(const ImageDatabase &,
                        const std::pair<const std::string &, const std::string &> &format);
        void writeFrame(const cv::Mat &frame, Entry &entry) override;
        const std::string &getVideoFileName() const;

    private:
        cv::VideoWriter m_Writer;
        const std::string m_FileName;
    };

    //! Base class for GridRecorder and RouteRecorder
    template<class FrameWriterType>
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
        const std::vector<std::string> m_ExtraFieldNames;
        ImageDatabase &m_ImageDatabase;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        template<class T, class... Ts>
        Recorder(ImageDatabase &imageDatabase,
                 bool isRoute,
                 T &&format,
                 std::vector<std::string> extraFieldNames)
          : FrameWriterType{ imageDatabase, std::move(format) }
          , m_ExtraFieldNames(std::move(extraFieldNames))
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
               << std::setw(2) << (creationTime.tm_mon + 1) << "-"
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
#ifdef BOB_IS_EXPERIMENT
                   << "is_experiment" << 1
#else
                   << "is_experiment" << 0
#endif
                   << "type" << (isRoute ? "route" : "grid");
        }

        template<class... Ts>
        void addEntry(const cv::Mat &image,
                      const Vector3<millimeter_t> &position,
                      const degree_t heading,
                      const std::array<size_t, 3> &gridPosition,
                      Ts&&... extraFieldValues)
        {
            // **NOTE** for reasons I can't quite comprehend, "BOB_ASSERT(sizeof...(extraFieldValues) == m_ExtraFieldNames.size());"
            // results in "error C2065: 'extraFieldValues': undeclared identifier" on Visual Studio
            constexpr size_t numExtraFields = sizeof...(extraFieldValues);
            BOB_ASSERT(numExtraFields == m_ExtraFieldNames.size());
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
    class GridRecorder
      : public Recorder<ImageFileWriter> {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange,
                     const Range &yrange, const Range &zrange = Range(0_mm),
                     degree_t heading = 0_deg,
                     std::string imageFormat = "png",
                     std::vector<std::string> extraFieldNames = {});

        std::string getCurrentFilenameRoot() const override;

        //! Get the physical position represented by grid coordinates
        Vector3<millimeter_t> getPosition(const std::array<size_t, 3> &gridPosition) const;

        //! Get a vector of all possible positions for this grid
        std::vector<Vector3<millimeter_t>> getPositions() const;

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

        size_t maximumSize() const;
        size_t sizeX() const;
        size_t sizeY() const;
        size_t sizeZ() const;

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        const std::array<size_t, 3> m_Size;
        std::array<size_t, 3> m_Current;
    };

    //! For saving images recorded along a route
    template<class FrameWriterType>
    class RouteRecorder
      : public Recorder<FrameWriterType> {
    public:
        template<class T>
        RouteRecorder(ImageDatabase &imageDatabase, T &&format,
                      std::vector<std::string> extraFieldNames)
          : Recorder<FrameWriterType>(imageDatabase, true,
                                      std::move(format),
                                      std::move(extraFieldNames))
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
    std::vector<cv::Mat> loadImages(const cv::Size &size = {}, size_t frameSkip = 1,
                                    bool greyscale = true) const;

    //! Load all of the images in this database into the specified std::vector<>
    void loadImages(std::vector<cv::Mat> &images, const cv::Size &size = {},
                    size_t frameSkip = 1, bool greyscale = true) const;

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Get the time at which this database was created
    const std::tm &getCreationTime() const;

    //! Start recording a grid of images
    GridRecorder getGridRecorder(const Range &xrange, const Range &yrange,
                                 const Range &zrange = Range(0_mm),
                                 degree_t heading = 0_deg,
                                 std::string imageFormat = "png",
                                 std::vector<std::string> extraFieldNames = {});

    //! Start recording a route
    RouteRecorder<ImageFileWriter> getRouteRecorder(std::string imageFormat = "png",
                                                    std::vector<std::string> extraFieldNames = {});

    /**!
     * \brief Start recording a route, saving images into video file using
     *        default AVI/MJPEG format.
     */
    RouteRecorder<VideoFileWriter> getRouteVideoRecorder(const cv::Size &resolution,
                                                         hertz_t fps,
                                                         std::vector<std::string> extraFieldNames = {});

    //! Start recording a route, saving images into video file with a custom codec
    RouteRecorder<VideoFileWriter> getRouteVideoRecorder(const cv::Size &resolution,
                                                         hertz_t fps,
                                                         const std::string &extension,
                                                         const std::string &codec,
                                                         std::vector<std::string> extraFieldNames = {});

    hertz_t getFrameRate() const;

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    template<class Func>
    void forEachImage(const Func &func, size_t frameSkip = 1,
                      bool greyscale = true) const
    {
        BOB_ASSERT(frameSkip > 0);

        if (empty()) {
            return;
        }

        // If database consists of individual image files...
        if (m_VideoFilePath.empty()) {
            tbb::parallel_for(tbb::blocked_range<size_t>(0, m_Entries.size() / frameSkip),
                              [&](const auto &r) {
                                  for (size_t i = r.begin(); i != r.end(); ++i) {
                                      func(i, m_Entries[i * frameSkip].load(greyscale));
                                  }
                              });

            return;
        }

        // ...otherwise we have a video file
        cv::VideoCapture cap{ m_VideoFilePath.str() };
        BOB_ASSERT(cap.isOpened());

        cv::Mat img;
        for (size_t i = 0; i < m_Entries.size() / frameSkip; i++) {
            BOB_ASSERT(cap.read(img));

            if (greyscale) {
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
            }

            func(i, img);

            /*
             * It is possible to explicitly jump to a given frame with OpenCV,
             * but that turns out to be reeeeeeeaaaally slow.
             */
            for (size_t j = 1; j < frameSkip && cap.grab(); j++)
                ;
        }
    }

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination,
                const cv::Size &unwrapRes,
                size_t frameSkip = 1,
                bool greyscale = false) const;

    //! Return true if fn1 should be sorted before fn2
    static bool fileNameCompare(const std::string &fn1, const std::string &fn2);

private:
    filesystem::path m_Path, m_VideoFilePath;
    std::vector<Entry> m_Entries;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    std::tm m_CreationTime;
    hertz_t m_FrameRate{ 0 };
    bool m_IsRoute;
    bool m_NeedsUnwrapping = true;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    ImageDatabase(const std::tm *creationTime, filesystem::path databasePath,
                  bool overwrite);

    void generateUnwrapCSV(const filesystem::path &destination, size_t frameSkip) const;
    void loadMetadata();
    bool loadCSV();
    bool readDirectoryEntries();
    void writeImage(const std::string &filename, const cv::Mat &image) const;

    void addNewEntries(std::vector<Entry> &newEntries,
                       const std::vector<std::string> &extraFieldNames);
}; // ImageDatabase
} // Navigation
} // BoB robotics
